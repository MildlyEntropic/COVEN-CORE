// SPDX-License-Identifier: MIT
//! protocol.rs — COVEN Protocol Message Definitions
//!
//! Defines message types for dock-rover communication.
//! The rover is "dumb" - it collects raw sensor data and uploads batches to dock.
//!
//! ## Communication Architecture
//!
//! **There is NO wireless communication.** Communication occurs ONLY via UART
//! when the rover is physically docked via the COVEN Type-A 9-pin connector.
//! During missions, the rover operates completely autonomously with no comms.
//!
//! Responsibilities:
//! - Define rover state enumeration
//! - Define dock-to-rover message types
//! - Define rover-to-dock message types
//! - Define raw sensor data structures for batch upload
//! - Implement wire format serialization/deserialization
//!
//! Wire format (per Interface Spec v0.3 — COBS):
//!   [LEN_HI] [LEN_LO] [COBS-encoded data] [0x00]
//! See dock_uart.rs for framing implementation.
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Third-party ---
use serde::{Deserialize, Serialize};

// --- Local ---
use crate::utils::now_secs;

// ------------------------
// --- State Enumeration ---
// ------------------------

/// Rover state in the COVEN protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RoverState {
    /// Initial hardware initialization.
    Boot,
    /// Waiting for dock identification request.
    Identify,
    /// Waiting for dock verification request.
    WaitVerify,
    /// Idle, ready for tasks.
    Normal,
    /// Executing a mission in the field.
    FieldOps,
    /// Rejected by dock.
    Rejected,
    /// Lost connection to dock.
    Disconnected,
}

impl std::fmt::Display for RoverState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Boot => write!(f, "BOOT"),
            Self::Identify => write!(f, "IDENTIFY"),
            Self::WaitVerify => write!(f, "WAIT_VERIFY"),
            Self::Normal => write!(f, "NORMAL"),
            Self::FieldOps => write!(f, "FIELD_OPS"),
            Self::Rejected => write!(f, "REJECTED"),
            Self::Disconnected => write!(f, "DISCONNECTED"),
        }
    }
}

// ------------------------
// --- Message Types ---
// ------------------------

/// Message types for dock-to-rover communication.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DockMessage {
    /// Dock requests rover identification.
    IdentifyReq {
        /// Dock identifier.
        dock_id: String,
        /// Coven name (e.g., "The_Graeae").
        dock_name: String,
        /// Suggested name (empty if asking rover to claim identity).
        assigned_name: String,
    },
    /// Dock confirms rover identity.
    IdentifyAck {
        /// Dock identifier.
        dock_id: String,
        /// Final assigned name (may differ from claimed).
        assigned_name: String,
        /// Welcome message.
        message: String,
    },
    /// Dock requests verification (or reports verify result).
    VerifyReq {
        /// Dock identifier.
        dock_id: String,
        /// Target module identifier.
        module_id: String,
        /// Whether the dock accepted this module (false for VERIFY_FAIL).
        accepted: bool,
    },
    /// Dock assigns a task to the rover.
    TaskReq {
        /// Dock identifier.
        dock_id: String,
        /// Target module identifier.
        module_id: String,
        /// Unique task identifier.
        task_id: String,
        /// Task type (e.g., "explore").
        task: String,
        /// Waypoints to visit.
        waypoints: Vec<Waypoint>,
        /// Dock X position for return.
        dock_x: f64,
        /// Dock Y position for return.
        dock_y: f64,
        /// Coverage threshold for task completion.
        coverage_threshold: f64,
        /// Task timeout in seconds.
        timeout: f64,
    },
    /// Dock enables charging power.
    EnablePower {
        /// Voltage in volts.
        voltage: u32,
        /// Duration in seconds (0 = indefinite).
        duration: f64,
    },
    /// Manual velocity command.
    CmdVel {
        /// Linear velocity in m/s.
        linear: f64,
        /// Angular velocity in rad/s.
        angular: f64,
    },
}

/// Message types for rover-to-dock communication.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RoverMessage {
    /// Rover identification response.
    IdentifyRep {
        /// Module identifier.
        module_id: String,
        /// Module type (e.g., "ReconRover").
        module_type: String,
        /// Firmware version.
        firmware: String,
        /// Current battery level percentage.
        battery_level: f64,
        /// Status message.
        status: String,
        /// Hardware capability bitmask (0x01=encoders, 0x02=lidar, 0x04=ultrasonic, etc.)
        capabilities: u8,
    },
    /// Rover verification response.
    VerifyRep {
        /// Module identifier.
        module_id: String,
        /// Whether all checks passed.
        success: bool,
        /// List of failed checks.
        failed_checks: Vec<String>,
        /// Additional note.
        note: String,
    },
    /// Periodic heartbeat message.
    Heartbeat {
        /// Module identifier.
        module_id: String,
        /// Battery percentage.
        battery_pct: f64,
        /// Mission status string.
        mission_status: String,
        /// X position in meters.
        x: f64,
        /// Y position in meters.
        y: f64,
        /// Heading in radians.
        theta: f64,
    },
    /// Task acknowledgment.
    TaskAck {
        /// Module identifier.
        module_id: String,
        /// Task identifier.
        task_id: String,
        /// Whether task was accepted.
        success: bool,
    },
    /// Task start notification.
    TaskStart {
        /// Module identifier.
        module_id: String,
        /// Task identifier.
        task_id: String,
        /// Start timestamp.
        timestamp: f64,
    },
    /// Task completion notification.
    TaskComplete {
        /// Module identifier.
        module_id: String,
        /// Task identifier.
        task_id: String,
        /// Whether task succeeded.
        success: bool,
        /// Map data (empty - sent via DataBatch).
        map_data: String,
        /// Coverage percentage.
        coverage: f64,
        /// Duration in seconds.
        duration: f64,
    },
    /// Batch upload of raw sensor data.
    DataBatch {
        /// Module identifier.
        module_id: String,
        /// Mission identifier.
        mission_id: String,
        /// Raw sensor batch.
        batch: SensorBatch,
    },
    /// Real-time scan data (mock/debug mode only).
    ScanData {
        /// Module identifier.
        module_id: String,
        /// Compact scan data.
        scan: ScanDataCompact,
    },
    /// Real-time odom data (mock/debug mode only).
    OdomData {
        /// Module identifier.
        module_id: String,
        /// Compact odometry data.
        odom: OdomDataCompact,
    },
    /// Fault alert (rover → dock).
    #[allow(dead_code)] // Protocol-defined, not yet generated
    FaultAlert {
        /// Module identifier.
        module_id: String,
        /// Fault description.
        fault: String,
    },
}

/// Waypoint for navigation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    /// X position in meters.
    pub x: f64,
    /// Y position in meters.
    pub y: f64,
    /// Target heading in radians.
    pub yaw: f64,
    /// Position tolerance in meters.
    pub tolerance: f64,
}

// ------------------------
// --- Sensor Types ---
// ------------------------

/// Sensor type tags for batch data. The transport layer treats sensor data
/// as opaque bytes — only the dock's processing layer interprets the payload.
#[allow(dead_code)] // Protocol constants — future sensor types
pub const SENSOR_TYPE_NONE: u8 = 0x00;
pub const SENSOR_TYPE_LIDAR: u8 = 0x01;
#[allow(dead_code)]
pub const SENSOR_TYPE_CAMERA: u8 = 0x02;
#[allow(dead_code)]
pub const SENSOR_TYPE_SPECTROMETER: u8 = 0x03;
#[allow(dead_code)]
pub const SENSOR_TYPE_DRILL: u8 = 0x04;

// ------------------------
// --- Raw Sensor Data ---
// ------------------------

/// A single raw sensor sample recorded during a mission.
///
/// Sensor data is stored as an opaque byte blob. The `sensor_type` field
/// on the parent `SensorBatch` tells the dock how to interpret it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawSensorSample {
    /// Timestamp in seconds since mission start.
    pub timestamp: f64,
    /// Left wheel encoder ticks since last sample.
    pub left_ticks: i32,
    /// Right wheel encoder ticks since last sample.
    pub right_ticks: i32,
    /// Opaque sensor payload — format depends on batch sensor_type.
    pub sensor_data: Vec<u8>,
}

/// A batch of sensor samples from a mission.
///
/// The batch carries a `sensor_type` tag and an opaque `sensor_config` blob
/// so the dock knows how to interpret the sensor data without hardcoding
/// any particular sensor format into the transport layer.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorBatch {
    /// When the mission started (Unix timestamp).
    pub mission_start: f64,
    /// Wheel radius in millimeters.
    pub wheel_radius_mm: u16,
    /// Wheel base in millimeters.
    pub wheel_base_mm: u16,
    /// Encoder ticks per revolution.
    pub ticks_per_rev: u16,
    /// What kind of sensor data the samples carry.
    pub sensor_type: u8,
    /// Sensor-specific calibration/configuration (opaque to transport).
    pub sensor_config: Vec<u8>,
    /// The raw sensor samples.
    pub samples: Vec<RawSensorSample>,
}

impl SensorBatch {
    /// Create a new empty batch with robot and sensor configuration.
    pub fn new(
        wheel_radius_mm: u16,
        wheel_base_mm: u16,
        ticks_per_rev: u16,
        sensor_type: u8,
        sensor_config: Vec<u8>,
    ) -> Self {
        Self {
            mission_start: now_secs(),
            wheel_radius_mm,
            wheel_base_mm,
            ticks_per_rev,
            sensor_type,
            sensor_config,
            samples: Vec::new(),
        }
    }

    /// Add a sample to the batch.
    pub fn add_sample(&mut self, sample: RawSensorSample) {
        self.samples.push(sample);
    }

    /// Get the number of samples.
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Check if batch is empty.
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Clear all samples (keep configuration).
    #[allow(dead_code)]
    pub fn clear(&mut self) {
        self.samples.clear();
        self.mission_start = now_secs();
    }
}

// ------------------------
// --- LiDAR Encoding ---
// ------------------------

/// Encode LiDAR calibration into sensor_config bytes.
///
/// Format: [angle_min:f64 LE][angle_max:f64 LE][num_rays:u16 LE] = 18 bytes.
pub fn encode_lidar_config(angle_min: f64, angle_max: f64, num_rays: u16) -> Vec<u8> {
    let mut config = Vec::with_capacity(18);
    config.extend_from_slice(&angle_min.to_le_bytes());
    config.extend_from_slice(&angle_max.to_le_bytes());
    config.extend_from_slice(&num_rays.to_le_bytes());
    config
}

/// Decode LiDAR calibration from sensor_config bytes.
#[allow(dead_code)]
pub fn decode_lidar_config(config: &[u8]) -> Option<(f64, f64, u16)> {
    if config.len() < 18 {
        return None;
    }
    let angle_min = f64::from_le_bytes(config[0..8].try_into().ok()?);
    let angle_max = f64::from_le_bytes(config[8..16].try_into().ok()?);
    let num_rays = u16::from_le_bytes(config[16..18].try_into().ok()?);
    Some((angle_min, angle_max, num_rays))
}

/// Encode LiDAR ranges (u16 mm) into sensor_data bytes.
pub fn encode_lidar_ranges(ranges_mm: &[u16]) -> Vec<u8> {
    let mut data = Vec::with_capacity(ranges_mm.len() * 2);
    for &r in ranges_mm {
        data.extend_from_slice(&r.to_le_bytes());
    }
    data
}

// ------------------------
// --- Legacy Compact Formats ---
// ------------------------

/// Compact scan data for transmission (legacy - mock mode only).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanDataCompact {
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// Minimum scan angle in radians.
    pub angle_min: f64,
    /// Maximum scan angle in radians.
    pub angle_max: f64,
    /// Angle between consecutive rays in radians.
    pub angle_increment: f64,
    /// Minimum detectable range in meters.
    pub range_min: f64,
    /// Maximum detectable range in meters.
    pub range_max: f64,
    /// Ranges encoded as u16 millimeters to save bandwidth.
    pub ranges_mm: Vec<u16>,
}

/// Compact odometry data for transmission (legacy - mock mode only).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OdomDataCompact {
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// X position in meters.
    pub x: f64,
    /// Y position in meters.
    pub y: f64,
    /// Heading in radians.
    pub theta: f64,
    /// Linear velocity in m/s.
    pub v_linear: f64,
    /// Angular velocity in rad/s.
    pub v_angular: f64,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl ScanDataCompact {
    /// Create compact scan data from a LaserScan.
    pub fn from_scan(scan: &crate::lidar::LaserScan) -> Self {
        Self {
            timestamp: scan.timestamp,
            angle_min: scan.angle_min,
            angle_max: scan.angle_max,
            angle_increment: scan.angle_increment,
            range_min: scan.range_min,
            range_max: scan.range_max,
            ranges_mm: scan.to_ranges_mm(),
        }
    }
}

impl OdomDataCompact {
    /// Create compact odometry data from an Odometry reading.
    pub fn from_odom(odom: &crate::hardware::encoders::Odometry) -> Self {
        Self {
            timestamp: odom.timestamp,
            x: odom.x,
            y: odom.y,
            theta: odom.theta,
            v_linear: odom.v_linear,
            v_angular: odom.v_angular,
        }
    }
}

