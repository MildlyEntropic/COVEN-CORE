// SPDX-License-Identifier: MIT
//! main.rs — COVEN Rover Daemon
//!
//! Lightweight rover controller for COVEN-compliant reconnaissance modules.
//! No ROS2 dependency - controls hardware directly via rppal.
//!
//! ## IMPORTANT: Communication Architecture
//!
//! **There is NO wireless communication (WiFi, Ethernet, RF) on COVEN rovers.**
//!
//! The rover communicates with the dock ONLY when physically docked via the
//! COVEN Type-A 9-pin connector. Communication uses UART over the connector's
//! data lines. When deployed on a mission, the rover operates completely
//! autonomously with no communication link to the dock.
//!
//! This is a **data-mule architecture** per COVEN Interface Spec v0.2:
//! - Rover docks physically and receives mission via UART
//! - Rover undocks and executes mission autonomously (NO COMMUNICATION)
//! - Rover returns to dock and uploads sensor data via UART
//!
//! Responsibilities:
//! - Parse command-line arguments and load configuration
//! - Initialize hardware subsystems (motors, encoders, LiDAR, battery)
//! - Run hardware diagnostics when requested
//! - Launch main control loop (real or mock mode)
//!
//! Usage:
//!   coven-rover              # Run with real hardware
//!   coven-rover --mock       # Run with simulated hardware (for testing)
//!   coven-rover -c config.toml  # Use specific config file
//!   coven-rover --diag       # Run hardware diagnostics and exit
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Module Declarations ---
// ------------------------

mod config;
mod diagnostics;
mod dock_uart;
mod hardware;
mod lidar;
mod mock;
mod navigation;
mod protocol;
mod state;
mod subsumption;
mod utils;

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::env;

// --- Third-party ---
use anyhow::Result;
use tracing::{error, info, warn, Level};
use tracing_subscriber::FmtSubscriber;

// --- Local ---
use crate::config::RoverConfig;
use crate::dock_uart::{DockUart, DockUartConfig};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Command line arguments parsed from argv.
struct Args {
    /// Run with simulated hardware instead of real GPIO/I2C.
    mock: bool,
    /// Path to custom TOML configuration file.
    config_path: Option<String>,
    /// Enable debug logging output.
    verbose: bool,
    /// Enable trace logging output (very verbose).
    trace: bool,
    /// Run hardware diagnostics and exit.
    diag: bool,
}

impl Args {
    /// Parse command line arguments from argv.
    fn parse() -> Self {
        let args: Vec<String> = env::args().collect();
        let mut mock = false;
        let mut config_path = None;
        let mut verbose = false;
        let mut trace = false;
        let mut diag = false;

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--mock" | "-m" => mock = true,
                "--verbose" | "-v" => verbose = true,
                "--trace" | "-vv" => trace = true,
                "--diag" | "-d" => diag = true,
                "--config" | "-c" => {
                    if i + 1 < args.len() {
                        config_path = Some(args[i + 1].clone());
                        i += 1;
                    }
                }
                "--help" | "-h" => {
                    println!("COVEN Rover Daemon\n");
                    println!("Usage: coven-rover [OPTIONS]\n");
                    println!("Options:");
                    println!("  -m, --mock         Run with simulated hardware");
                    println!("  -c, --config PATH  Use specific config file");
                    println!("  -v, --verbose      Enable debug logging");
                    println!("  -vv, --trace       Enable trace logging (very verbose)");
                    println!("  -d, --diag         Run hardware diagnostics and exit");
                    println!("  -h, --help         Show this help message");
                    std::process::exit(0);
                }
                _ => {}
            }
            i += 1;
        }

        Self {
            mock,
            config_path,
            verbose,
            trace,
            diag,
        }
    }
}

// ------------------------
// --- Entry Point ---
// ------------------------

/// Application entry point.
#[tokio::main]
async fn main() -> Result<()> {
    // Parse arguments
    let args = Args::parse();

    // Initialize logging with appropriate level
    let log_level = if args.trace {
        Level::TRACE
    } else if args.verbose || args.diag {
        Level::DEBUG
    } else {
        Level::INFO
    };

    FmtSubscriber::builder()
        .with_max_level(log_level)
        .with_target(false)
        .compact()
        .init();

    info!("╔══════════════════════════════════════════════════════════╗");
    info!(
        "║           COVEN Rover Daemon v{}            ║",
        env!("CARGO_PKG_VERSION")
    );
    info!("╚══════════════════════════════════════════════════════════╝");
    info!("Log level: {:?}", log_level);

    // Load configuration
    let config = if let Some(ref path) = args.config_path {
        info!("Loading config from: {}", path);
        RoverConfig::load_from_file(path).map_err(|e| {
            error!("Failed to load config file '{}': {}", path, e);
            e
        })?
    } else {
        info!("Loading default config (searching standard paths)...");
        RoverConfig::load_or_default()?
    };

    info!("--- Configuration ---");
    info!("  Rover ID: {}", config.rover_id);
    info!("  Coven: {}", config.coven_name);
    info!("  Dock UART: {} @ {} baud", config.dock_uart.port, config.dock_uart.baud_rate);
    info!("  Control rate: {} Hz", config.timing.control_rate);

    // Validate GPIO configuration
    info!("--- GPIO Pin Validation ---");
    let gpio_issues = config.validate_gpio_pins();
    if gpio_issues.is_empty() {
        info!("  [OK] All GPIO pin assignments valid");
    } else {
        for issue in &gpio_issues {
            error!("  [ERROR] {}", issue);
        }
        error!("");
        error!("!!! CONFIGURATION ERRORS DETECTED !!!");
        error!("  Fix the issues above in your config file.");
        error!("  GPIO conflicts will cause hardware initialization to fail.");

        if !args.diag {
            // In normal mode, fail fast on config errors
            return Err(anyhow::anyhow!(
                "Configuration validation failed: {} issues found",
                gpio_issues.len()
            ));
        }
        // In diagnostic mode, continue to help debug
        warn!("Continuing in diagnostic mode despite config errors...");
    }

    // Run diagnostic mode if requested
    if args.diag {
        return run_diagnostics(config).await;
    }

    // Create dock UART connection
    // NOTE: This is NOT WiFi/Ethernet. Communication only when physically docked.
    let dock_uart_config = DockUartConfig {
        port: config.dock_uart.port.clone(),
        baud_rate: config.dock_uart.baud_rate,
        retry_delay: std::time::Duration::from_secs_f64(config.dock_uart.retry_delay_secs),
        max_retries: config.dock_uart.max_retries,
    };
    let dock = DockUart::new(dock_uart_config);

    if args.mock {
        info!("Running in MOCK mode (simulated hardware)");
        info!("NOTE: UART dock communication will not work without physical hardware");
        run_mock_mode(config, dock).await
    } else {
        info!("Running with REAL hardware");
        info!("NOTE: Dock communication via UART on {} (NOT wireless)", config.dock_uart.port);
        run_real_mode(config, dock).await
    }
}

// ------------------------
// --- Mode Functions ---
// ------------------------

/// Run hardware diagnostics and exit.
async fn run_diagnostics(config: RoverConfig) -> Result<()> {
    use crate::hardware::BatteryReader;
    use crate::hardware::Hardware;
    use crate::lidar::LidarDriver;
    use std::time::Duration;

    info!("╔══════════════════════════════════════════════════════════╗");
    info!("║              HARDWARE DIAGNOSTIC MODE                    ║");
    info!("╚══════════════════════════════════════════════════════════╝");
    info!("");

    // Track test results for summary
    let mut test_results: Vec<(&str, &str, String)> = Vec::new();

    // Test 0: Configuration Summary
    info!("═══ TEST 0: Configuration Summary ═══");
    info!("  Rover ID: {}", config.rover_id);
    info!("  Coven: {}", config.coven_name);
    info!("  Dock UART: {} @ {} baud", config.dock_uart.port, config.dock_uart.baud_rate);
    info!("");
    info!("  GPIO Assignments:");
    info!(
        "    Motors: PWM L=GPIO{} R=GPIO{}",
        config.hardware.motors.left_pwm, config.hardware.motors.right_pwm
    );
    info!(
        "    Motor Dir L: IN1=GPIO{} IN2=GPIO{}",
        config.hardware.motors.left_in1, config.hardware.motors.left_in2
    );
    info!(
        "    Motor Dir R: IN1=GPIO{} IN2=GPIO{}",
        config.hardware.motors.right_in1, config.hardware.motors.right_in2
    );
    info!("    Standby: GPIO{}", config.hardware.motors.standby);
    info!(
        "    Encoder L: A=GPIO{} B=GPIO{}",
        config.hardware.encoders.left_a, config.hardware.encoders.left_b
    );
    info!(
        "    Encoder R: A=GPIO{} B=GPIO{}",
        config.hardware.encoders.right_a, config.hardware.encoders.right_b
    );
    info!("    LiDAR: {}", config.hardware.lidar.port);
    info!(
        "    Battery ADC: I2C 0x{:02X}",
        config.hardware.battery.adc_address
    );

    let gpio_issues = config.validate_gpio_pins();
    if gpio_issues.is_empty() {
        info!("[PASS] Configuration valid");
        test_results.push(("Config", "PASS", "All settings valid".to_string()));
    } else {
        for issue in &gpio_issues {
            error!("  [ERROR] {}", issue);
        }
        test_results.push(("Config", "FAIL", format!("{} issues", gpio_issues.len())));
    }

    // Test 1: GPIO / Motor / Encoder initialization
    info!("");
    info!("═══ TEST 1: Motor & Encoder Initialization ═══");
    let hardware_result = Hardware::new(&config);

    match hardware_result {
        Ok(mut hw) => {
            info!("[PASS] Hardware initialized successfully");
            test_results.push((
                "GPIO/PWM",
                "PASS",
                "Motors + encoders initialized".to_string(),
            ));

            // Test motor movement
            info!("");
            info!("═══ TEST 2: Motor Test ═══");
            info!("Testing left motor forward (0.5s)...");
            hw.motors.set_raw(0.3, 0.0);
            tokio::time::sleep(Duration::from_millis(500)).await;
            hw.motors.stop();

            info!("Testing right motor forward (0.5s)...");
            hw.motors.set_raw(0.0, 0.3);
            tokio::time::sleep(Duration::from_millis(500)).await;
            hw.motors.stop();

            info!("Testing both motors forward (0.5s)...");
            hw.motors.set_raw(0.3, 0.3);
            tokio::time::sleep(Duration::from_millis(500)).await;
            hw.motors.stop();

            info!("[INFO] Motor test complete - verify wheels moved correctly");
            test_results.push(("Motors", "INFO", "Manual verification required".to_string()));

            // Test encoder reading
            info!("");
            info!("═══ TEST 3: Encoder Test ═══");
            info!("Reading encoders for 3 seconds... (rotate wheels by hand)");
            let start = std::time::Instant::now();
            let mut last_ticks = (0i64, 0i64);
            while start.elapsed() < Duration::from_secs(3) {
                let (left, right) = hw.encoders.get_ticks();
                if left != last_ticks.0 || right != last_ticks.1 {
                    info!("  Encoder ticks: left={}, right={}", left, right);
                    last_ticks = (left, right);
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
            let (final_left, final_right) = hw.encoders.get_ticks();
            if final_left == 0 && final_right == 0 {
                warn!("[WARN] No encoder ticks detected - check wiring!");
                test_results.push(("Encoders", "WARN", "No ticks detected".to_string()));
            } else {
                info!(
                    "[PASS] Encoders responding: left={}, right={}",
                    final_left, final_right
                );
                test_results.push((
                    "Encoders",
                    "PASS",
                    format!("L={} R={}", final_left, final_right),
                ));
            }
        }
        Err(e) => {
            error!("[FAIL] Hardware initialization failed: {:#}", e);
            test_results.push(("GPIO/PWM", "FAIL", format!("{}", e)));
            test_results.push(("Motors", "SKIP", "Hardware init failed".to_string()));
            test_results.push(("Encoders", "SKIP", "Hardware init failed".to_string()));
            info!("");
            info!("Skipping motor and encoder tests due to init failure");
        }
    }

    // Test 4: LiDAR
    info!("");
    info!("═══ TEST 4: LiDAR Test ═══");
    let mut lidar = LidarDriver::new(&config.hardware.lidar);
    match lidar.start().await {
        Ok(()) => {
            info!("LiDAR started, waiting for scans (5 seconds)...");
            let start = std::time::Instant::now();
            let mut scan_count = 0;
            while start.elapsed() < Duration::from_secs(5) {
                if let Some(scan) = lidar.get_scan().await {
                    scan_count += 1;
                    let valid_rays = scan.ranges.iter().filter(|r| r.is_finite()).count();
                    info!(
                        "  Scan {}: {} total rays, {} valid, min={:.2}m, max={:.2}m",
                        scan_count,
                        scan.ranges.len(),
                        valid_rays,
                        scan.ranges
                            .iter()
                            .filter(|r| r.is_finite())
                            .cloned()
                            .fold(f64::INFINITY, f64::min),
                        scan.ranges
                            .iter()
                            .filter(|r| r.is_finite())
                            .cloned()
                            .fold(0.0, f64::max)
                    );
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
            lidar.stop().await;

            if scan_count == 0 {
                warn!("[WARN] No LiDAR scans received - check connection!");
                test_results.push(("LiDAR", "WARN", "No scans received".to_string()));
            } else {
                let hz = scan_count as f64 / 5.0;
                info!(
                    "[PASS] LiDAR working: {} scans in 5 seconds ({:.1} Hz)",
                    scan_count, hz
                );
                test_results.push(("LiDAR", "PASS", format!("{:.1} Hz", hz)));
            }
        }
        Err(e) => {
            error!("[FAIL] LiDAR start failed: {:#}", e);
            test_results.push(("LiDAR", "FAIL", format!("{}", e)));
        }
    }

    // Test 5: Battery ADC (optional - may not be present)
    info!("");
    info!("═══ TEST 5: Battery Monitor ═══");
    match BatteryReader::new(&config.hardware.battery) {
        Ok(mut battery) => match battery.read_percent() {
            Ok(percent) => {
                info!("[PASS] Battery: {:.0}%", percent);
                test_results.push(("Battery", "PASS", format!("{:.0}%", percent)));
            }
            Err(e) => {
                warn!("[WARN] Battery read failed: {}", e);
                test_results.push(("Battery", "WARN", "Read failed".to_string()));
            }
        },
        Err(e) => {
            warn!("[WARN] Battery ADC not available: {}", e);
            warn!("  (This is OK if no battery monitor is installed)");
            test_results.push(("Battery", "SKIP", "ADC not found".to_string()));
        }
    }

    // Test 6: UART connectivity to dock (via 9-pin connector)
    info!("");
    info!("═══ TEST 6: Dock UART Test ═══");
    info!(
        "  Target UART: {} @ {} baud",
        config.dock_uart.port, config.dock_uart.baud_rate
    );
    info!("  NOTE: Rover must be physically docked via 9-pin connector for this test");

    // Check if UART port exists
    let uart_path = std::path::Path::new(&config.dock_uart.port);
    if uart_path.exists() {
        info!("[PASS] UART device {} exists", config.dock_uart.port);
        test_results.push(("UART", "PASS", format!("Device {} found", config.dock_uart.port)));

        // Try to open the serial port briefly
        match tokio_serial::new(&config.dock_uart.port, config.dock_uart.baud_rate)
            .timeout(Duration::from_millis(100))
            .open()
        {
            Ok(_port) => {
                info!("[PASS] UART port opened successfully");
                info!("  (Full handshake test requires dock to be powered on)");
            }
            Err(e) => {
                warn!("[WARN] Cannot open UART port: {}", e);
                warn!("  (Port may be in use or require permissions)");
            }
        }
    } else {
        warn!("[WARN] UART device {} not found", config.dock_uart.port);
        warn!("  Ensure rover is physically docked via 9-pin connector");
        warn!("  Or check udev rules if using a symlink like /dev/ttyAMA0");
        test_results.push(("UART", "WARN", format!("Device {} not found", config.dock_uart.port)));
    }

    // Summary
    info!("");
    info!("╔══════════════════════════════════════════════════════════╗");
    info!("║              DIAGNOSTIC SUMMARY                          ║");
    info!("╚══════════════════════════════════════════════════════════╝");
    info!("");

    let mut pass_count = 0;
    let mut warn_count = 0;
    let mut fail_count = 0;
    let mut skip_count = 0;

    for (test, status, detail) in &test_results {
        let icon = match *status {
            "PASS" => {
                pass_count += 1;
                "✓"
            }
            "WARN" => {
                warn_count += 1;
                "⚠"
            }
            "FAIL" => {
                fail_count += 1;
                "✗"
            }
            "INFO" => "ℹ",
            _ => {
                skip_count += 1;
                "○"
            }
        };
        info!("  {} {:10} [{:4}] {}", icon, test, status, detail);
    }

    info!("");
    info!(
        "  Results: {} PASS, {} WARN, {} FAIL, {} SKIP",
        pass_count, warn_count, fail_count, skip_count
    );
    info!("");

    if fail_count > 0 {
        error!("  ╔════════════════════════════════════════════════════╗");
        error!("  ║  HARDWARE ISSUES DETECTED - Review errors above    ║");
        error!("  ╚════════════════════════════════════════════════════╝");
    } else if warn_count > 0 {
        warn!("  ╔════════════════════════════════════════════════════╗");
        warn!("  ║  Some warnings detected - Review output above      ║");
        warn!("  ╚════════════════════════════════════════════════════╝");
    } else {
        info!("  ╔════════════════════════════════════════════════════╗");
        info!("  ║  All tests passed! Hardware ready for operation.   ║");
        info!("  ╚════════════════════════════════════════════════════╝");
    }

    Ok(())
}

/// Run with real hardware on Raspberry Pi.
///
/// Note: `dock` is a UART connection via 9-pin connector, NOT wireless.
async fn run_real_mode(config: RoverConfig, dock: DockUart) -> Result<()> {
    use crate::hardware::Hardware;
    use crate::state::RoverStateMachine;
    use tokio::signal;

    // Initialize hardware
    let hardware = Hardware::new(&config)?;
    info!("Hardware initialized");

    // Create state machine
    let mut rover = RoverStateMachine::new(config, hardware, dock);

    // Run main loop with signal handling
    info!("Entering main loop (Ctrl+C to shutdown gracefully)");

    tokio::select! {
        result = rover.run() => {
            result
        }
        _ = signal::ctrl_c() => {
            info!("Shutdown signal received - stopping gracefully");
            // Hardware cleanup happens via Drop trait
            Ok(())
        }
    }
}

/// Run with mock hardware for desktop testing.
///
/// Note: In mock mode, we still use DockUart but it won't actually work
/// without the physical UART hardware - this is mainly for testing logic.
async fn run_mock_mode(config: RoverConfig, mut dock: DockUart) -> Result<()> {
    use crate::mock::MockHardware;
    use crate::navigation::{NavParams, NavState, VelocityCmd, WaypointFollower};
    use crate::protocol::{
        DockMessage, OdomDataCompact, RawSensorSample, RoverMessage, RoverState, ScanDataCompact,
        SensorBatch, SENSOR_TYPE_LIDAR, encode_lidar_config, encode_lidar_ranges,
    };
    use crate::subsumption::{LayerContext, SubsumptionArbiter};
    use std::time::{Duration, Instant};
    use tokio::signal;

    let mut hardware = MockHardware::new();
    let mut navigator = WaypointFollower::with_config(&config.navigation, config.timing.control_rate);
    let mut arbiter = SubsumptionArbiter::new(NavParams::from(&config.navigation));
    info!("Mock hardware initialized");
    info!("Press Ctrl+C to shutdown gracefully");

    // Connect to dock
    dock.connect().await?;
    info!("Connecting to dock...");

    let mut state = RoverState::Identify;
    let mut battery_pct = 100.0_f64;
    let mut last_heartbeat = Instant::now();
    let mut last_sample = Instant::now();

    // Witch name assigned by dock (set during handshake)
    let mut assigned_name: Option<String> = None;

    // Helper to get current module_id (assigned name or fallback to config)
    let module_id = |name: &Option<String>| -> String {
        name.as_ref()
            .cloned()
            .unwrap_or_else(|| config.rover_id.clone())
    };

    // Mission data (for batch upload on completion)
    let mut current_mission: Option<(String, SensorBatch)> = None;
    let mut mission_dock_x = 0.0_f64;
    let mut mission_dock_y = 0.0_f64;

    // Timing
    let loop_period = Duration::from_secs_f64(1.0 / config.timing.control_rate);
    let heartbeat_period = Duration::from_secs_f64(1.0 / config.timing.heartbeat_rate);
    let scan_period = Duration::from_millis(166); // ~6 Hz
    let sample_period = Duration::from_millis(100); // 10 Hz raw data collection
    let mut last_scan = Instant::now();

    loop {
        // Check for shutdown signal at start of each loop
        tokio::select! {
            biased;
            _ = signal::ctrl_c() => {
                info!("Shutdown signal received - stopping mock mode gracefully");
                hardware.stop();
                return Ok(());
            }
            _ = tokio::time::sleep(Duration::ZERO) => {
                // Continue with loop
            }
        }

        let loop_start = Instant::now();

        // Update mock hardware
        let odom = hardware.update();

        // Get scan periodically
        let scan = if last_scan.elapsed() >= scan_period {
            last_scan = Instant::now();
            Some(hardware.get_scan())
        } else {
            None
        };

        // Process incoming messages
        while let Some(msg) = dock.try_recv() {
            match msg {
                DockMessage::IdentifyReq {
                    dock_id,
                    dock_name,
                    assigned_name: name,
                } => {
                    info!(
                        dock_id = %dock_id,
                        dock_name = %dock_name,
                        assigned_name = %name,
                        "Received IDENTIFY_REQ - I am now {}!",
                        name
                    );

                    // Adopt the assigned witch name!
                    assigned_name = Some(name.clone());

                    let reply = RoverMessage::IdentifyRep {
                        module_id: name,
                        module_type: "MockRover".to_string(),
                        firmware: format!("{}-mock", env!("CARGO_PKG_VERSION")),
                        battery_level: battery_pct,
                        status: "OK".to_string(),
                        capabilities: 0x03, // ENCODERS + LIDAR
                    };
                    dock.send(reply).await?;

                    state = RoverState::WaitVerify;
                    info!("State -> WaitVerify");
                }

                DockMessage::VerifyReq {
                    dock_id,
                    module_id: recv_id,
                    accepted,
                } => {
                    let my_id = module_id(&assigned_name);
                    if recv_id != my_id {
                        continue;
                    }

                    info!(dock_id = %dock_id, module_id = %my_id, accepted = accepted, "Received VERIFY_REQ");

                    if !accepted {
                        warn!("Dock rejected verification");
                        state = RoverState::Rejected;
                        info!("State -> Rejected");
                        continue;
                    }

                    let reply = RoverMessage::VerifyRep {
                        module_id: my_id,
                        success: true,
                        failed_checks: Vec::new(),
                        note: "Mock mode - all checks pass".to_string(),
                    };
                    dock.send(reply).await?;

                    state = RoverState::Normal;
                    info!("State -> Normal (verified)");
                }

                DockMessage::CmdVel { linear, angular } => {
                    // Manual override - disable autonomous nav temporarily
                    hardware.set_velocity(linear, angular);
                }

                DockMessage::TaskReq {
                    task_id,
                    task,
                    waypoints,
                    dock_x,
                    dock_y,
                    ..
                } => {
                    let my_id = module_id(&assigned_name);
                    info!(
                        task_id = %task_id,
                        task = %task,
                        waypoints = waypoints.len(),
                        "{} received TASK_REQ",
                        my_id
                    );

                    let ack = RoverMessage::TaskAck {
                        module_id: my_id.clone(),
                        task_id: task_id.clone(),
                        success: true,
                    };
                    dock.send(ack).await?;

                    // Set up navigation waypoints
                    let wp_coords: Vec<(f64, f64)> = waypoints.iter().map(|w| (w.x, w.y)).collect();
                    navigator.set_waypoints(wp_coords);
                    navigator.set_dock_position(dock_x, dock_y);
                    mission_dock_x = dock_x;
                    mission_dock_y = dock_y;

                    // Create sensor batch for raw data
                    let batch = SensorBatch::new(
                        hardware.wheel_radius_mm(),
                        hardware.wheel_base_mm(),
                        hardware.ticks_per_rev(),
                        SENSOR_TYPE_LIDAR,
                        encode_lidar_config(-std::f64::consts::PI, std::f64::consts::PI, 360),
                    );
                    current_mission = Some((task_id.clone(), batch));

                    let now = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_secs_f64();

                    let start = RoverMessage::TaskStart {
                        module_id: my_id,
                        task_id,
                        timestamp: now,
                    };
                    dock.send(start).await?;

                    state = RoverState::FieldOps;
                    info!("State -> FieldOps (autonomous navigation)");
                }

                DockMessage::EnablePower { voltage, duration } => {
                    info!(
                        voltage = voltage,
                        duration = duration,
                        "ENABLE_POWER (mock - no-op)"
                    );
                }

                DockMessage::IdentifyAck {
                    dock_id,
                    assigned_name: name,
                    message,
                } => {
                    // Dock confirmed our identity (may be different from what we claimed)
                    info!(
                        dock_id = %dock_id,
                        assigned_name = %name,
                        message = %message,
                        "Received IDENTIFY_ACK - final name: {}",
                        name
                    );

                    // Adopt the assigned name (may differ from what we claimed)
                    assigned_name = Some(name);
                    state = RoverState::WaitVerify;
                    info!("State -> WaitVerify");
                }
            }
        }

        // State-specific behavior
        match state {
            RoverState::FieldOps => {
                // Record raw sensor sample
                if last_sample.elapsed() >= sample_period {
                    if let Some((_, ref mut batch)) = current_mission {
                        let (left_ticks, right_ticks) = hardware.get_delta_ticks();
                        let sensor_data = scan.as_ref()
                            .map(|s| encode_lidar_ranges(&s.to_ranges_mm()))
                            .unwrap_or_default();

                        batch.add_sample(RawSensorSample {
                            timestamp: odom.timestamp - batch.mission_start,
                            left_ticks,
                            right_ticks,
                            sensor_data,
                        });
                    }
                    last_sample = Instant::now();
                }

                // === SUBSUMPTION NAVIGATION ===

                // Pre-compute attractive velocity from WaypointFollower
                // (goal-seeking only — L0 handles obstacle avoidance)
                let nav_cmd = navigator.update(
                    odom.x, odom.y, odom.theta,
                );
                let has_goal = navigator.has_goal();

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
                    d_safe: config.navigation.d_safe,
                    battery_pct,
                    low_battery_threshold: config.navigation.low_battery_threshold,
                    nav_cmd,
                    has_goal,
                    has_mission: current_mission.is_some(),
                    dock_x: mission_dock_x,
                    dock_y: mission_dock_y,
                };

                // Arbiter decides final velocity (L0–L4 cascade)
                let cmd = arbiter.evaluate(&ctx);
                hardware.set_velocity(cmd.linear, cmd.angular);

                // Check navigation state
                match navigator.state() {
                    NavState::Arrived => {
                        let my_id = module_id(&assigned_name);
                        info!("{} navigation complete - mission successful", my_id);
                        if let Some((task_id, batch)) = current_mission.take() {
                            let now = std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .unwrap_or_default()
                                .as_secs_f64();
                            let duration = now - batch.mission_start;

                            let batch_msg = RoverMessage::DataBatch {
                                module_id: my_id.clone(),
                                mission_id: task_id.clone(),
                                batch,
                            };
                            if let Err(e) = dock.send(batch_msg).await {
                                warn!("Failed to upload sensor batch: {}", e);
                            }

                            let complete = RoverMessage::TaskComplete {
                                module_id: my_id,
                                task_id,
                                success: true,
                                map_data: "".to_string(),
                                coverage: 0.0,
                                duration,
                            };
                            if let Err(e) = dock.send(complete).await {
                                warn!("Failed to send TaskComplete: {}", e);
                            }
                        }
                        navigator.clear();
                        hardware.stop();
                        state = RoverState::Normal;
                    }
                    NavState::Stuck => {
                        let my_id = module_id(&assigned_name);
                        info!("{} navigation stuck - aborting mission", my_id);
                        if let Some((task_id, batch)) = current_mission.take() {
                            let now = std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .unwrap_or_default()
                                .as_secs_f64();
                            let duration = now - batch.mission_start;

                            let batch_msg = RoverMessage::DataBatch {
                                module_id: my_id.clone(),
                                mission_id: task_id.clone(),
                                batch,
                            };
                            if let Err(e) = dock.send(batch_msg).await {
                                warn!("Failed to upload sensor batch: {}", e);
                            }

                            let complete = RoverMessage::TaskComplete {
                                module_id: my_id,
                                task_id,
                                success: false,
                                map_data: "".to_string(),
                                coverage: 0.0,
                                duration,
                            };
                            if let Err(e) = dock.send(complete).await {
                                warn!("Failed to send TaskComplete: {}", e);
                            }
                        }
                        navigator.clear();
                        hardware.stop();
                        state = RoverState::Normal;
                    }
                    _ => {}
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
                    d_safe: config.navigation.d_safe,
                    battery_pct,
                    low_battery_threshold: config.navigation.low_battery_threshold,
                    nav_cmd: VelocityCmd::stop(),
                    has_goal: false,
                    has_mission: false,
                    dock_x: 0.0,
                    dock_y: 0.0,
                };

                let cmd = arbiter.evaluate(&ctx);
                hardware.set_velocity(cmd.linear, cmd.angular);
            }

            _ => {
                // Not in FieldOps or Normal - motors should already be stopped
                hardware.stop();
            }
        }

        // Stream real-time sensor data (mock mode visualization)
        // Only send when a new scan is available (~6Hz) to avoid exceeding
        // 115200 baud UART bandwidth with 20Hz OdomData+ScanData.
        if (state == RoverState::FieldOps || state == RoverState::Normal) && scan.is_some() {
            let odom_msg = RoverMessage::OdomData {
                module_id: module_id(&assigned_name),
                odom: OdomDataCompact::from_odom(&odom),
            };
            let _ = dock.send(odom_msg).await;

            if let Some(ref s) = scan {
                let scan_msg = RoverMessage::ScanData {
                    module_id: module_id(&assigned_name),
                    scan: ScanDataCompact::from_scan(s),
                };
                let _ = dock.send(scan_msg).await;
            }
        }

        // Simulate battery drain
        if state == RoverState::FieldOps {
            battery_pct -= 0.001;
            battery_pct = battery_pct.max(0.0);
        }

        // Send heartbeat
        if last_heartbeat.elapsed() >= heartbeat_period {
            let mission_status = match state {
                RoverState::FieldOps => "ACTIVE",
                RoverState::Normal => "IDLE",
                _ => "STARTUP",
            };

            let msg = RoverMessage::Heartbeat {
                module_id: module_id(&assigned_name),
                battery_pct,
                mission_status: mission_status.to_string(),
                x: odom.x,
                y: odom.y,
                theta: odom.theta,
            };
            dock.send(msg).await?;
            last_heartbeat = Instant::now();
        }

        // Maintain loop rate
        let elapsed = loop_start.elapsed();
        if elapsed < loop_period {
            tokio::time::sleep(loop_period - elapsed).await;
        }
    }
}
