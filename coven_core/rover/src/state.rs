// SPDX-License-Identifier: MIT
//! state.rs — COVEN Rover State Machine
//!
//! Implements the rover-side FSM for the COVEN protocol lifecycle.
//! Coordinates hardware, navigation, and dock communication.
//!
//! ## IMPORTANT: Communication Architecture
//!
//! **There is NO wireless communication on COVEN rovers.**
//!
//! The rover communicates with the dock ONLY when physically docked via the
//! COVEN Type-A 9-pin connector. UART is used over the connector's data lines.
//! When deployed, the rover operates completely autonomously with no link to dock.
//!
//! Lifecycle (per Interface Spec v0.2):
//!   DISCONNECTED → DETECTED → IDENTIFY → VERIFIED → ENABLED → NORMAL_OPERATIONS
//!
//! Responsibilities:
//! - Manage state transitions (Boot → Identify → WaitVerify → Normal → FieldOps)
//! - Handle incoming dock messages and generate responses via UART (when docked)
//! - Coordinate Lyapunov-based navigation during missions (autonomous, no comms)
//! - Collect raw sensor data for batch upload to dock (when docked)
//! - Monitor battery and enforce safety timeouts
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};

// --- Third-party ---
use anyhow::Result;
use tracing::{debug, info, warn};

// --- Local ---
use crate::config::RoverConfig;
use crate::dock_uart::DockUart;
use crate::hardware::{BatteryReader, Hardware};
use crate::lidar::LidarDriver;
use crate::navigation::{NavParams, NavState, VelocityCmd, WaypointFollower};
use crate::subsumption::{LayerContext, SubsumptionArbiter};
use crate::protocol::{
    DockMessage, RawSensorSample, RoverMessage, RoverState, SensorBatch,
    SENSOR_TYPE_LIDAR, encode_lidar_config, encode_lidar_ranges,
};
use crate::utils::now_secs;

// ------------------------
// --- Constants ---
// ------------------------

/// Path to the file where the rover's assigned name is persisted.
const NAME_PERSISTENCE_FILE: &str = "/var/lib/coven/witch_name";

// ------------------------
// --- Data Structures ---
// ------------------------

/// Mission data containing raw sensor batch for upload to dock.
struct MissionData {
    /// Unique task identifier assigned by dock.
    task_id: String,
    /// Raw sensor samples collected during mission.
    batch: SensorBatch,
    /// Dock X position for return navigation.
    dock_x: f64,
    /// Dock Y position for return navigation.
    dock_y: f64,
    /// Mission start time for timeout tracking.
    start_time: Instant,
    /// Estimated mission timeout based on path distance with safety factor.
    timeout_secs: f64,
}

/// Main rover state machine coordinating all subsystems.
///
/// ## Communication Note
///
/// The `dock` field is a UART connection, NOT WiFi/Ethernet.
/// Communication only occurs when physically docked via the 9-pin connector.
/// When deployed on a mission, the rover operates autonomously with no comms.
pub struct RoverStateMachine {
    /// Rover configuration loaded from TOML.
    config: RoverConfig,
    /// Hardware interface (motors, encoders).
    hardware: Hardware,
    /// LiDAR driver for obstacle detection.
    lidar: LidarDriver,
    /// Cached last valid LiDAR scan (LiDAR runs at ~6Hz, control at 20Hz).
    last_scan: Option<crate::lidar::LaserScan>,
    /// When the last valid LiDAR scan was received.
    last_scan_time: Instant,
    /// Whether a batch-full warning has been logged this mission.
    batch_full_warned: bool,
    /// UART connection to dock (via 9-pin connector, NOT wireless).
    dock: DockUart,
    /// Optional battery reader (may not be present).
    battery: Option<BatteryReader>,

    /// Current FSM state.
    state: RoverState,
    /// Time when current state was entered (for timeouts).
    state_entered_at: Instant,
    /// Current battery percentage.
    battery_pct: f64,

    /// Witch name assigned by dock (e.g., "Morgan_Le_Fay").
    assigned_name: Option<String>,

    /// Lyapunov-based waypoint navigator.
    navigator: WaypointFollower,
    /// Subsumption behavioral arbiter (L0–L4 per thesis).
    arbiter: SubsumptionArbiter,

    /// Last heartbeat send time.
    last_heartbeat: Instant,
    /// Last sensor sample time.
    last_sample: Instant,
    /// Last battery read time.
    last_battery_read: Instant,

    /// Current mission data (if in FieldOps).
    current_mission: Option<MissionData>,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl RoverStateMachine {
    /// Create a new rover state machine with the given configuration.
    ///
    /// Note: `dock` is a UART connection via the 9-pin connector, NOT wireless.
    pub fn new(config: RoverConfig, hardware: Hardware, dock: DockUart) -> Self {
        let lidar = LidarDriver::new(&config.hardware.lidar);

        // Try to initialize battery reader (may fail if ADC not present)
        let mut battery = BatteryReader::new(&config.hardware.battery).ok();
        let initial_battery = battery
            .as_mut()
            .and_then(|b| b.read_percent().ok())
            .unwrap_or(100.0);

        // Create navigator with config params
        let navigator = WaypointFollower::with_config(&config.navigation, config.timing.control_rate);

        // Create subsumption arbiter with nav params
        let arbiter = SubsumptionArbiter::new(NavParams::from(&config.navigation));

        // Try to load persisted witch name from previous session
        let persisted_name = Self::load_persisted_name();
        if let Some(ref name) = persisted_name {
            info!("Loaded persisted witch name: {}", name);
        }

        Self {
            config,
            hardware,
            lidar,
            last_scan: None,
            last_scan_time: Instant::now(),
            batch_full_warned: false,
            dock,
            battery,
            state: RoverState::Boot,
            state_entered_at: Instant::now(),
            battery_pct: initial_battery,
            assigned_name: persisted_name, // Load from previous session if available
            navigator,
            arbiter,
            last_heartbeat: Instant::now(),
            last_sample: Instant::now(),
            last_battery_read: Instant::now(),
            current_mission: None,
        }
    }

    /// Load persisted witch name from disk.
    fn load_persisted_name() -> Option<String> {
        match fs::read_to_string(NAME_PERSISTENCE_FILE) {
            Ok(name) => {
                let name = name.trim().to_string();
                if name.is_empty() {
                    None
                } else {
                    Some(name)
                }
            }
            Err(_) => None,
        }
    }

    /// Save assigned witch name to disk for persistence across reboots.
    fn save_persisted_name(name: &str) -> Result<()> {
        // Create parent directory if it doesn't exist
        if let Some(parent) = Path::new(NAME_PERSISTENCE_FILE).parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(NAME_PERSISTENCE_FILE, name)?;
        info!("Persisted witch name to {}", NAME_PERSISTENCE_FILE);
        Ok(())
    }

    /// Get the rover's current name (assigned by dock, or fallback to config).
    fn module_id(&self) -> &str {
        self.assigned_name
            .as_deref()
            .unwrap_or(&self.config.rover_id)
    }

    /// Get the name to claim during identification.
    fn claimed_name(&self) -> &str {
        self.assigned_name.as_deref().unwrap_or("new_witch")
    }

    /// Main run loop executing the state machine.
    pub async fn run(&mut self) -> Result<()> {
        // Start LiDAR
        self.lidar.start().await?;
        info!("LiDAR started");

        // Connect to dock
        self.dock.connect().await?;
        info!("Connecting to dock...");

        // Transition to IDENTIFY state
        self.transition_to(RoverState::Identify);

        // Control loop timing
        let loop_period = Duration::from_secs_f64(1.0 / self.config.timing.control_rate);
        let heartbeat_period = Duration::from_secs_f64(1.0 / self.config.timing.heartbeat_rate);
        loop {
            let loop_start = Instant::now();

            // Update odometry
            let odom = self.hardware.encoders.update();

            // Get latest LiDAR scan (non-blocking), cache for arbiter
            let has_fresh_scan = if let Some(new_scan) = self.lidar.get_scan().await {
                self.last_scan = Some(new_scan);
                self.last_scan_time = Instant::now();
                true
            } else {
                false
            };

            // If no scan for >2s, LiDAR may have crashed — clear cache
            if self.last_scan_time.elapsed() > Duration::from_secs(2) {
                if self.last_scan.is_some() {
                    warn!("LiDAR data stale >2s — possible sensor failure, stopping navigation");
                    self.last_scan = None;
                }
            }

            // Update battery reading periodically
            // Use configurable interval (more frequent during missions)
            let battery_interval = if self.state == RoverState::FieldOps {
                Duration::from_secs_f64(self.config.timing.battery_check_interval)
            } else {
                Duration::from_secs(5) // Less frequent when idle
            };

            if self.last_battery_read.elapsed() >= battery_interval {
                if let Some(ref mut battery) = self.battery {
                    // block_in_place: I2C reads block ~7ms (thread::sleep + rppal).
                    // Tells tokio to move other tasks off this thread while we block.
                    let pct = tokio::task::block_in_place(|| battery.read_percent());
                    if let Ok(pct) = pct {
                        self.battery_pct = pct;
                    }
                }
                self.last_battery_read = Instant::now();
            }

            // Process incoming messages
            while let Some(msg) = self.dock.try_recv() {
                self.handle_dock_message(msg).await?;
            }

            // Reference cached scan after message handling (borrow checker)
            let scan = &self.last_scan;

            // State-specific behavior
            match self.state {
                RoverState::Boot => {
                    // Transition to Identify once hardware is ready
                    self.transition_to(RoverState::Identify);
                }

                RoverState::Identify => {
                    // Wait for IDENTIFY_REQ from dock
                    // (handled in handle_dock_message)

                    // Timeout after 30 seconds - dock didn't send IDENTIFY_REQ
                    if self.state_entered_at.elapsed() > Duration::from_secs(30) {
                        warn!("Identify timeout - no response from dock, reconnecting");
                        self.dock.disconnect().await;
                        self.dock.connect().await?;
                        self.state_entered_at = Instant::now();
                    }
                }

                RoverState::WaitVerify => {
                    // Wait for VERIFY_REQ from dock
                    // (handled in handle_dock_message)

                    // Timeout after 30 seconds - dock may have missed our response
                    if self.state_entered_at.elapsed() > Duration::from_secs(30) {
                        warn!("WaitVerify timeout - returning to Identify state");
                        self.transition_to(RoverState::Identify);
                    }
                }

                RoverState::Normal => {
                    // No mission — arbiter runs L2 (wander) / L1 (return to dock)

                    let min_range = scan.as_ref()
                        .map(|s| s.ranges.iter()
                            .filter(|r| r.is_finite() && **r > 0.0)
                            .cloned()
                            .fold(f64::INFINITY, f64::min))
                        .unwrap_or(f64::INFINITY);

                    let ctx = LayerContext {
                        robot_x: odom.x,
                        robot_y: odom.y,
                        robot_theta: odom.theta,
                        lidar_ranges: scan.as_ref()
                            .map(|s| s.ranges.clone())
                            .unwrap_or_default(),
                        min_range,
                        lidar_angle_min: scan.as_ref()
                            .map(|s| s.angle_min).unwrap_or(0.0),
                        lidar_angle_increment: scan.as_ref()
                            .map(|s| s.angle_increment).unwrap_or(0.0),
                        d_safe: self.config.navigation.d_safe,
                        battery_pct: self.battery_pct,
                        low_battery_threshold: self.config.navigation.low_battery_threshold,
                        nav_cmd: VelocityCmd::stop(),
                        has_goal: false,
                        has_mission: false,
                        dock_x: 0.0,
                        dock_y: 0.0,
                    };

                    let cmd = self.arbiter.evaluate(&ctx);
                    self.hardware.motors.set_velocity(cmd.linear, cmd.angular);
                }

                RoverState::FieldOps => {
                    // Executing a mission with Lyapunov navigation

                    // === SAFETY CHECKS ===

                    // Check for low battery - emergency return to dock
                    if self.battery_pct < self.config.navigation.low_battery_threshold {
                        warn!(
                            battery_pct = self.battery_pct,
                            threshold = self.config.navigation.low_battery_threshold,
                            "LOW BATTERY - aborting mission and returning to dock"
                        );
                        self.complete_mission(false).await?;
                        continue; // Skip rest of FieldOps processing
                    }

                    // Check for mission timeout
                    if let Some(ref mission) = self.current_mission {
                        let elapsed = mission.start_time.elapsed().as_secs_f64();
                        if elapsed > mission.timeout_secs {
                            warn!(
                                elapsed_secs = elapsed,
                                timeout_secs = mission.timeout_secs,
                                "MISSION TIMEOUT - aborting and returning to dock"
                            );
                            self.complete_mission(false).await?;
                            continue;
                        }
                    }

                    // === SENSOR DATA COLLECTION ===

                    // Record raw sensor sample periodically (~10Hz)
                    // Use fresh_scan (not cached) to avoid re-recording same scan
                    let sample_period = Duration::from_millis(100);
                    if self.last_sample.elapsed() >= sample_period {
                        if let Some(ref mut mission) = self.current_mission {
                            // Get encoder ticks (raw data)
                            let (left_ticks, right_ticks) =
                                self.hardware.encoders.get_delta_ticks();

                            // Get sensor data as opaque bytes (only from fresh scans)
                            let sensor_data = if has_fresh_scan {
                                self.last_scan.as_ref()
                                    .map(|s| encode_lidar_ranges(&s.to_ranges_mm()))
                                    .unwrap_or_default()
                            } else {
                                Vec::new()
                            };

                            let sample = RawSensorSample {
                                timestamp: odom.timestamp - mission.batch.mission_start,
                                left_ticks,
                                right_ticks,
                                sensor_data,
                            };
                            if !mission.batch.add_sample(sample) && !self.batch_full_warned {
                                warn!(
                                    "Sensor batch full ({} samples) — no more data will be recorded",
                                    mission.batch.len()
                                );
                                self.batch_full_warned = true;
                            }
                        }
                        self.last_sample = Instant::now();
                    }

                    // === SUBSUMPTION NAVIGATION ===

                    // Pre-compute attractive velocity from WaypointFollower
                    // (goal-seeking only — L0 handles obstacle avoidance)
                    let nav_cmd = self.navigator.update(
                        odom.x, odom.y, odom.theta,
                    );
                    let has_goal = self.navigator.has_goal();

                    // Build subsumption layer context
                    let min_range = scan.as_ref()
                        .map(|s| s.ranges.iter()
                            .filter(|r| r.is_finite() && **r > 0.0)
                            .cloned()
                            .fold(f64::INFINITY, f64::min))
                        .unwrap_or(f64::INFINITY);

                    let ctx = LayerContext {
                        robot_x: odom.x,
                        robot_y: odom.y,
                        robot_theta: odom.theta,
                        lidar_ranges: scan.as_ref()
                            .map(|s| s.ranges.clone())
                            .unwrap_or_default(),
                        min_range,
                        lidar_angle_min: scan.as_ref()
                            .map(|s| s.angle_min).unwrap_or(0.0),
                        lidar_angle_increment: scan.as_ref()
                            .map(|s| s.angle_increment).unwrap_or(0.0),
                        d_safe: self.config.navigation.d_safe,
                        battery_pct: self.battery_pct,
                        low_battery_threshold: self.config.navigation.low_battery_threshold,
                        nav_cmd,
                        has_goal,
                        has_mission: self.current_mission.is_some(),
                        dock_x: self.current_mission.as_ref()
                            .map(|m| m.dock_x).unwrap_or(0.0),
                        dock_y: self.current_mission.as_ref()
                            .map(|m| m.dock_y).unwrap_or(0.0),
                    };

                    // Arbiter decides final velocity (L0–L4 cascade)
                    let cmd = self.arbiter.evaluate(&ctx);
                    self.hardware.motors.set_velocity(cmd.linear, cmd.angular);

                    // Check if navigation completed
                    match self.navigator.state() {
                        NavState::Arrived => {
                            info!("Navigation complete - mission successful");
                            self.complete_mission(true).await?;
                        }
                        NavState::Stuck => {
                            warn!("Navigation stuck - aborting mission");
                            self.complete_mission(false).await?;
                        }
                        _ => {}
                    }
                }

                RoverState::Rejected => {
                    // Error state - stop motors and wait for manual intervention
                    self.hardware.motors.stop();
                    // After 60 seconds, retry verification
                    if self.state_entered_at.elapsed() > Duration::from_secs(60) {
                        info!("Rejected state timeout - retrying identification");
                        self.transition_to(RoverState::Identify);
                    }
                }

                RoverState::Disconnected => {
                    // Disconnected from dock - stop motors and attempt reconnection
                    self.hardware.motors.stop();
                    // After 5 seconds, try to reconnect
                    if self.state_entered_at.elapsed() > Duration::from_secs(5) {
                        info!("Attempting reconnection to dock...");
                        if self.dock.connect().await.is_ok() {
                            self.transition_to(RoverState::Identify);
                        } else {
                            // Reset timer and try again
                            self.state_entered_at = Instant::now();
                        }
                    }
                }
            }

            // Send heartbeat if needed (don't crash loop on send failure)
            if self.last_heartbeat.elapsed() >= heartbeat_period {
                if let Err(e) = self.send_heartbeat(&odom).await {
                    warn!("Failed to send heartbeat: {}", e);
                }
                self.last_heartbeat = Instant::now();
            }

            // Sleep to maintain loop rate
            let elapsed = loop_start.elapsed();
            if elapsed < loop_period {
                tokio::time::sleep(loop_period - elapsed).await;
            } else {
                debug!(elapsed_ms = elapsed.as_millis(), "Control loop overrun");
            }
        }
    }

    /// Handle a message from the dock.
    async fn handle_dock_message(&mut self, msg: DockMessage) -> Result<()> {
        match msg {
            DockMessage::IdentifyReq {
                dock_id,
                dock_name,
                assigned_name,
            } => {
                // Two cases:
                // 1. assigned_name is empty: Dock is asking what we claim to be
                // 2. assigned_name is set: Legacy flow, dock is telling us our name

                if assigned_name.is_empty() {
                    // New protocol: Dock wants to know our claimed identity
                    let claimed = self.claimed_name().to_string();
                    info!(
                        dock_id = %dock_id,
                        dock_name = %dock_name,
                        claimed_name = %claimed,
                        "Received IDENTIFY_REQ - claiming identity as {}",
                        claimed
                    );

                    // Send IDENTIFY_REP with our claimed name
                    // Dock will respond with IDENTIFY_ACK confirming our final name
                    let reply = RoverMessage::IdentifyRep {
                        module_id: claimed,
                        module_type: "ReconRover".to_string(),
                        firmware: env!("CARGO_PKG_VERSION").to_string(),
                        battery_level: self.battery_pct,
                        status: "OK".to_string(),
                        capabilities: 0x03, // ENCODERS + LIDAR
                    };
                    self.dock.send(reply).await?;

                    // Stay in Identify state, waiting for IDENTIFY_ACK
                    self.transition_to(RoverState::Identify);
                } else {
                    // Legacy protocol: Dock is directly assigning us a name
                    info!(
                        dock_id = %dock_id,
                        dock_name = %dock_name,
                        assigned_name = %assigned_name,
                        "Received IDENTIFY_REQ (legacy) - I am now {}!",
                        assigned_name
                    );

                    // Adopt and persist the assigned witch name
                    self.assigned_name = Some(assigned_name.clone());
                    if let Err(e) = Self::save_persisted_name(&assigned_name) {
                        warn!("Failed to persist witch name: {}", e);
                    }

                    // Send IDENTIFY_REP echoing back our name
                    let reply = RoverMessage::IdentifyRep {
                        module_id: assigned_name,
                        module_type: "ReconRover".to_string(),
                        firmware: env!("CARGO_PKG_VERSION").to_string(),
                        battery_level: self.battery_pct,
                        status: "OK".to_string(),
                        capabilities: 0x03, // ENCODERS + LIDAR
                    };
                    self.dock.send(reply).await?;

                    self.transition_to(RoverState::WaitVerify);
                }
            }

            DockMessage::IdentifyAck {
                dock_id,
                assigned_name,
                message,
            } => {
                // Dock confirms our identity (may be different from what we claimed)
                let claimed = self.claimed_name();
                if assigned_name != claimed {
                    info!(
                        dock_id = %dock_id,
                        claimed = %claimed,
                        assigned = %assigned_name,
                        message = %message,
                        "Dock reassigned our name: {} -> {}",
                        claimed, assigned_name
                    );
                } else {
                    info!(
                        dock_id = %dock_id,
                        assigned = %assigned_name,
                        message = %message,
                        "Dock confirmed our identity: {}",
                        assigned_name
                    );
                }

                // Adopt and persist the final assigned name
                self.assigned_name = Some(assigned_name.clone());
                if let Err(e) = Self::save_persisted_name(&assigned_name) {
                    warn!("Failed to persist witch name: {}", e);
                }

                self.transition_to(RoverState::WaitVerify);
            }

            DockMessage::VerifyReq { dock_id, module_id, accepted } => {
                if module_id != self.module_id() {
                    return Ok(()); // Not for us
                }

                info!(dock_id = %dock_id, module_id = %module_id, accepted = accepted, "Received VERIFY_REQ");

                if !accepted {
                    warn!("Dock rejected verification for {}", module_id);
                    self.transition_to(RoverState::Rejected);
                    return Ok(());
                }

                // Run self-checks
                let (success, failed_checks) = self.run_self_checks();

                let reply = RoverMessage::VerifyRep {
                    module_id: self.module_id().to_string(),
                    success,
                    failed_checks,
                    note: if success {
                        "All systems nominal".to_string()
                    } else {
                        "Some checks failed".to_string()
                    },
                };
                self.dock.send(reply).await?;

                if success {
                    self.transition_to(RoverState::Normal);
                } else {
                    self.transition_to(RoverState::Rejected);
                }
            }

            DockMessage::TaskReq {
                dock_id: _,
                module_id,
                task_id,
                task,
                waypoints,
                dock_x,
                dock_y,
                coverage_threshold: _,
                timeout: _,
            } => {
                if module_id != self.module_id() && !module_id.is_empty() {
                    return Ok(()); // Not for us
                }

                info!(
                    task_id = %task_id,
                    task = %task,
                    waypoints = waypoints.len(),
                    "{} received TASK_REQ",
                    self.module_id()
                );

                // Acknowledge task
                let ack = RoverMessage::TaskAck {
                    module_id: self.module_id().to_string(),
                    task_id: task_id.clone(),
                    success: true,
                };
                self.dock.send(ack).await?;

                // Set up navigation waypoints. Honor each waypoint's
                // declared tolerance (Waypoint::tolerance) by passing it
                // through to the navigator instead of dropping it. Without
                // this, the rover converges to the navigator's tighter
                // default tolerance (typically 0.10 m) and can limit-cycle
                // when the waypoint actually wanted 0.30 m of slack.
                let wp_coords: Vec<(f64, f64)> =
                    waypoints.iter().map(|w| (w.x, w.y)).collect();
                let wp_with_tol: Vec<(f64, f64, f64)> = waypoints
                    .iter()
                    .map(|w| (w.x, w.y, w.tolerance))
                    .collect();
                self.navigator.set_waypoints_with_tolerances(wp_with_tol);
                // Add dock as final destination (return home, default tolerance).
                self.navigator.set_dock_position(dock_x, dock_y);

                // Calculate total path distance for timeout estimation
                // Path: current position → all waypoints → dock
                let mut total_distance = 0.0;
                let current_odom = self.hardware.encoders.get_odometry();
                let mut prev_x = current_odom.x;
                let mut prev_y = current_odom.y;

                for (wx, wy) in &wp_coords {
                    let dx = wx - prev_x;
                    let dy = wy - prev_y;
                    total_distance += (dx * dx + dy * dy).sqrt();
                    prev_x = *wx;
                    prev_y = *wy;
                }
                // Add return path to dock
                let dx = dock_x - prev_x;
                let dy = dock_y - prev_y;
                total_distance += (dx * dx + dy * dy).sqrt();

                // Estimate time: distance / max_speed, with safety factor
                // safety_factor of 5.0 = 500% of direct-line time
                // (accounts for obstacle avoidance, turning, etc.)
                let max_speed = self.config.navigation.max_linear;
                let direct_time = if max_speed > 0.0 {
                    total_distance / max_speed
                } else {
                    300.0
                };
                let timeout_secs = direct_time * self.config.navigation.mission_timeout_factor;

                info!(
                    path_distance_m = total_distance,
                    direct_time_secs = direct_time,
                    timeout_secs = timeout_secs,
                    "Mission timeout calculated"
                );

                // Create sensor batch for raw data collection
                // Get robot config from hardware
                let batch = SensorBatch::new(
                    self.hardware.encoders.wheel_radius_mm(),
                    self.hardware.encoders.wheel_base_mm(),
                    self.hardware.encoders.ticks_per_rev(),
                    SENSOR_TYPE_LIDAR,
                    encode_lidar_config(-std::f64::consts::PI, std::f64::consts::PI, 360),
                );

                self.current_mission = Some(MissionData {
                    task_id: task_id.clone(),
                    batch,
                    dock_x,
                    dock_y,
                    start_time: Instant::now(),
                    timeout_secs,
                });
                self.batch_full_warned = false;

                // Send task start
                let start = RoverMessage::TaskStart {
                    module_id: self.module_id().to_string(),
                    task_id,
                    timestamp: now_secs(),
                };
                self.dock.send(start).await?;

                self.transition_to(RoverState::FieldOps);
            }

            DockMessage::EnablePower { voltage, duration } => {
                info!(
                    voltage = voltage,
                    duration_secs = duration,
                    "Received ENABLE_POWER - entering charging mode"
                );

                // Power control is dock-initiated during charging
                // The rover should:
                // 1. Stop all motors for safety during charging
                // 2. Reduce activity to conserve power
                // 3. Duration of 0.0 means "until further notice"

                // Safety: Stop motors during charging
                self.hardware.motors.stop();

                // Log the charging parameters
                if voltage == 0 {
                    info!("Charging disabled - resuming normal operation");
                } else {
                    info!(
                        "Charging enabled at {}V for {:.1}s",
                        voltage,
                        if duration == 0.0 {
                            f64::INFINITY
                        } else {
                            duration
                        }
                    );
                    // Could implement: Set a flag to prevent motor commands during charging
                    // Could implement: GPIO relay control if hardware supports it
                }
            }

            DockMessage::CmdVel { linear, angular } => {
                debug!(linear = linear, angular = angular, "Received CMD_VEL");
                self.hardware.motors.set_velocity(linear, angular);
            }
        }

        Ok(())
    }

    /// Run self-diagnostic checks.
    fn run_self_checks(&self) -> (bool, Vec<String>) {
        let mut failed = Vec::new();

        // Check battery level
        if self.battery_pct < 10.0 {
            failed.push("battery_low".to_string());
        }

        // Could add more checks:
        // - Motor responsiveness
        // - Encoder readings
        // - LiDAR connectivity

        (failed.is_empty(), failed)
    }

    /// Send heartbeat message to dock.
    async fn send_heartbeat(&self, odom: &crate::hardware::encoders::Odometry) -> Result<()> {
        let mission_status = match self.state {
            RoverState::FieldOps => "ACTIVE",
            RoverState::Normal => "IDLE",
            _ => "STARTUP",
        };

        let msg = RoverMessage::Heartbeat {
            module_id: self.module_id().to_string(),
            battery_pct: self.battery_pct,
            mission_status: mission_status.to_string(),
            x: odom.x,
            y: odom.y,
            theta: odom.theta,
        };

        self.dock.send(msg).await
    }

    /// Transition to a new state.
    fn transition_to(&mut self, new_state: RoverState) {
        info!(from = %self.state, to = %new_state, "State transition");
        self.state = new_state;
        self.state_entered_at = Instant::now();
    }

    /// Complete current mission and return to Normal state.
    pub async fn complete_mission(&mut self, success: bool) -> Result<()> {
        // Stop navigation
        self.navigator.clear();
        self.hardware.motors.stop();

        if let Some(mission) = self.current_mission.take() {
            let duration = now_secs() - mission.batch.mission_start;
            let num_samples = mission.batch.len();

            info!(
                task_id = %mission.task_id,
                samples = num_samples,
                duration_secs = duration,
                "{} mission complete - uploading sensor batch",
                self.module_id()
            );

            // Upload raw sensor batch to dock
            // This is the key part - rover is dumb, just uploads raw data
            // Dock will run SLAM on it
            // Note: UART may be disconnected if rover hasn't re-docked yet.
            // Gracefully handle send failures — don't crash the daemon.
            let batch_msg = RoverMessage::DataBatch {
                module_id: self.module_id().to_string(),
                mission_id: mission.task_id.clone(),
                batch: mission.batch,
            };
            if let Err(e) = self.dock.send(batch_msg).await {
                warn!(error = %e, "Failed to upload sensor batch — UART may be disconnected");
            }

            // Send completion message
            let complete = RoverMessage::TaskComplete {
                module_id: self.module_id().to_string(),
                task_id: mission.task_id,
                success,
                map_data: "".to_string(), // Data sent via DataBatch
                coverage: 0.0,            // Dock calculates this from SLAM
                duration,
            };
            if let Err(e) = self.dock.send(complete).await {
                warn!(error = %e, "Failed to send TaskComplete — UART may be disconnected");
            }
        }

        self.transition_to(RoverState::Normal);
        Ok(())
    }
}
