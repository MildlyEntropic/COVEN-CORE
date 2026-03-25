// SPDX-License-Identifier: MIT
//! config.rs — COVEN Rover Configuration
//!
//! Defines configuration structures for all rover subsystems.
//! Supports loading from TOML files with sensible defaults.
//!
//! ## IMPORTANT: Communication Architecture
//!
//! **There is NO wireless communication (WiFi, Ethernet, RF) on COVEN rovers.**
//!
//! Rovers communicate with the dock ONLY when physically connected via the
//! COVEN Type-A 9-pin connector. Communication uses UART over the connector's
//! data lines. The rover operates completely autonomously when deployed.
//!
//! This is a **data-mule architecture**: rovers collect sensor data during
//! missions and upload it to the dock via UART when they return and dock.
//!
//! See: COVEN Interface Specification v0.2 (20250808.ShultisAnder.COVEN.CAD.InterfaceSpec.pdf)
//!
//! Responsibilities:
//! - Define motor driver pin assignments (TB6612FNG)
//! - Define encoder pin assignments and parameters
//! - Define LiDAR serial port configuration
//! - Define dock UART communication parameters
//! - Define battery monitoring parameters
//! - Define navigation tuning parameters
//! - Define control loop timing
//! - Validate GPIO pin assignments for conflicts
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::path::Path;

// --- Third-party ---
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Main configuration structure for the rover.
///
/// ## Communication Note
///
/// COVEN rovers do NOT use WiFi, Ethernet, or any wireless communication.
/// Communication with the dock occurs ONLY via UART when physically docked.
/// See `dock_uart` field for UART configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct RoverConfig {
    /// Unique rover identifier (e.g., "Morgan_Le_Fay")
    pub rover_id: String,

    /// COVEN network name (e.g., "The_Graeae")
    pub coven_name: String,

    /// Dock UART communication configuration.
    /// NOTE: This is NOT WiFi/Ethernet. Communication only occurs when
    /// physically docked via the 9-pin COVEN connector.
    pub dock_uart: DockUartConfig,

    /// Hardware configuration
    pub hardware: HardwareConfig,

    /// Control loop timing
    pub timing: TimingConfig,

    /// Navigation parameters
    pub navigation: NavigationConfig,
}

/// Hardware pin and parameter configuration.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HardwareConfig {
    /// Motor driver configuration.
    pub motors: MotorConfig,
    /// Encoder configuration.
    pub encoders: EncoderConfig,
    /// LiDAR configuration.
    pub lidar: LidarConfig,
    /// Battery monitoring configuration.
    pub battery: BatteryConfig,
}

/// TB6612FNG motor driver pin configuration.
///
/// 4-wheel skid-steer layout with 2× TB6612FNG drivers:
/// - Driver 1: front-left (channel A) + front-right (channel B)
/// - Driver 2: rear-left (channel A) + rear-right (channel B)
///
/// For normal skid-steer driving, FL=RL and FR=RR (same command per side).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct MotorConfig {
    // --- TB6612FNG #1: Front motors ---
    /// Front-left motor PWM pin (BCM numbering).
    pub front_left_pwm: u8,
    /// Front-left motor IN1 pin.
    pub front_left_in1: u8,
    /// Front-left motor IN2 pin.
    pub front_left_in2: u8,
    /// Front-right motor PWM pin.
    pub front_right_pwm: u8,
    /// Front-right motor IN1 pin.
    pub front_right_in1: u8,
    /// Front-right motor IN2 pin.
    pub front_right_in2: u8,
    /// Standby pin for driver 1 (HIGH to enable).
    pub standby_1: u8,

    // --- TB6612FNG #2: Rear motors ---
    /// Rear-left motor PWM pin.
    pub rear_left_pwm: u8,
    /// Rear-left motor IN1 pin.
    pub rear_left_in1: u8,
    /// Rear-left motor IN2 pin.
    pub rear_left_in2: u8,
    /// Rear-right motor PWM pin.
    pub rear_right_pwm: u8,
    /// Rear-right motor IN1 pin.
    pub rear_right_in1: u8,
    /// Rear-right motor IN2 pin.
    pub rear_right_in2: u8,
    /// Standby pin for driver 2 (HIGH to enable).
    pub standby_2: u8,

    /// PWM frequency in Hz.
    pub pwm_frequency: f64,
    /// Wheel separation in meters.
    pub wheel_base: f64,
    /// Wheel radius in meters.
    pub wheel_radius: f64,
    /// Maximum motor RPM.
    pub max_rpm: f64,
}

/// Quadrature encoder configuration for 4-wheel skid-steer.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct EncoderConfig {
    /// Front-left encoder channel A pin.
    pub front_left_a: u8,
    /// Front-left encoder channel B pin.
    pub front_left_b: u8,
    /// Front-right encoder channel A pin.
    pub front_right_a: u8,
    /// Front-right encoder channel B pin.
    pub front_right_b: u8,
    /// Rear-left encoder channel A pin.
    pub rear_left_a: u8,
    /// Rear-left encoder channel B pin.
    pub rear_left_b: u8,
    /// Rear-right encoder channel A pin.
    pub rear_right_a: u8,
    /// Rear-right encoder channel B pin.
    pub rear_right_b: u8,
    /// Pulses per revolution (after gearbox).
    pub pulses_per_rev: u32,
}

/// RPLIDAR C1 configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct LidarConfig {
    /// Serial port device path.
    pub port: String,
    /// Baud rate.
    pub baud_rate: u32,
    /// Scan rate in Hz.
    pub scan_rate: f64,
    /// Minimum range in meters.
    pub range_min: f64,
    /// Maximum range in meters.
    pub range_max: f64,
}

/// Battery monitoring configuration via I2C ADC.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct BatteryConfig {
    /// I2C address of ADC (ADS1015/ADS1115).
    pub adc_address: u8,
    /// Voltage divider ratio (actual = measured * ratio).
    pub divider_ratio: f64,
    /// Full battery voltage (100%).
    pub full_voltage: f64,
    /// Empty battery voltage (0%).
    pub empty_voltage: f64,
}

/// Dock UART communication configuration.
///
/// ## IMPORTANT: NO WIRELESS COMMUNICATION
///
/// COVEN rovers do NOT use WiFi, Ethernet, RF, or any wireless protocol.
/// Communication with the dock occurs ONLY via UART when the rover is
/// physically connected to the dock via the COVEN Type-A 9-pin connector.
///
/// The rover operates completely autonomously during missions with no
/// communication link to the dock. This is the **data-mule architecture**.
///
/// Protocol: COVEN Interface Specification v0.2
/// - Frame format: [0x7E] [TYPE] [LEN] [PAYLOAD] [CRC] [0x7F]
/// - Physical layer: UART via 9-pin connector pins 7/8 (ID/Sense lines)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct DockUartConfig {
    /// Serial port device path for dock communication.
    /// This is the UART connected to the 9-pin dock connector.
    /// Common: "/dev/ttyAMA0" (Pi GPIO UART) or "/dev/ttyS0"
    pub port: String,
    /// Baud rate for dock communication.
    pub baud_rate: u32,
    /// Retry delay between connection attempts (seconds).
    pub retry_delay_secs: f64,
    /// Maximum retry attempts before giving up.
    pub max_retries: u32,
}

/// Control loop timing configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct TimingConfig {
    /// Main control loop rate in Hz.
    pub control_rate: f64,
    /// Heartbeat send rate in Hz.
    pub heartbeat_rate: f64,
    /// Odometry publish rate in Hz.
    /// Note: currently unused — odometry updates at control_rate.
    /// Retained for config file compatibility.
    #[allow(dead_code)]
    pub odom_rate: f64,
    /// Command velocity timeout in seconds.
    pub cmd_timeout: f64,
    /// Battery check interval during FieldOps in seconds.
    pub battery_check_interval: f64,
}

/// Navigation parameters for Lyapunov potential field control.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct NavigationConfig {
    /// Attractive gain - how strongly to pull toward goal.
    pub k_att: f64,
    /// Repulsive gain - how strongly to push away from obstacles.
    pub k_rep: f64,
    /// Obstacle influence distance in meters.
    pub d_influence: f64,
    /// Minimum safe distance in meters (emergency stop threshold).
    pub d_safe: f64,
    /// Goal reached tolerance in meters.
    pub goal_tolerance: f64,
    /// Maximum linear velocity in m/s.
    pub max_linear: f64,
    /// Maximum angular velocity in rad/s.
    pub max_angular: f64,
    /// Heading alignment gain.
    pub k_heading: f64,
    /// Low battery threshold (%) - trigger return to dock.
    pub low_battery_threshold: f64,
    /// Mission timeout safety factor (multiplier on estimated time).
    pub mission_timeout_factor: f64,
}

// ------------------------
// --- Default Implementations ---
// ------------------------

impl Default for RoverConfig {
    fn default() -> Self {
        Self {
            rover_id: "unnamed_rover".to_string(),
            coven_name: "The_Graeae".to_string(),
            dock_uart: DockUartConfig::default(),
            hardware: HardwareConfig::default(),
            timing: TimingConfig::default(),
            navigation: NavigationConfig::default(),
        }
    }
}

impl Default for DockUartConfig {
    fn default() -> Self {
        Self {
            // UART connected to 9-pin dock connector
            // NOTE: This is NOT WiFi or Ethernet. Communication only when docked.
            port: "/dev/ttyAMA0".to_string(), // Pi GPIO UART
            baud_rate: 115200,
            retry_delay_secs: 1.0,
            max_retries: 10,
        }
    }
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            // TB6612FNG #1 — Front motors (BCM numbering)
            front_left_pwm: 12,   // HW PWM0
            front_left_in1: 5,
            front_left_in2: 6,
            front_right_pwm: 13,  // HW PWM1
            front_right_in1: 16,
            front_right_in2: 26,
            standby_1: 17,

            // TB6612FNG #2 — Rear motors (BCM numbering)
            rear_left_pwm: 18,    // pigpio DMA PWM
            rear_left_in1: 19,
            rear_left_in2: 20,
            rear_right_pwm: 21,   // pigpio DMA PWM
            rear_right_in1: 25,
            rear_right_in2: 8,
            standby_2: 7,

            pwm_frequency: 1000.0,
            wheel_base: 0.298,   // CubeRover 2U wheel separation
            wheel_radius: 0.0325, // 65mm diameter wheels
            max_rpm: 130.0,      // JGA25-371 ~130 RPM no-load
        }
    }
}

impl Default for EncoderConfig {
    fn default() -> Self {
        Self {
            front_left_a: 23,
            front_left_b: 24,
            front_right_a: 27,
            front_right_b: 22,
            rear_left_a: 9,
            rear_left_b: 10,
            rear_right_a: 11,
            rear_right_b: 4,
            pulses_per_rev: 312, // JGA25-371, 26:1 gearbox, 12 PPR
        }
    }
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            port: "/dev/rplidar".to_string(),
            baud_rate: 460800,
            scan_rate: 5.5,
            range_min: 0.15,
            range_max: 12.0,
        }
    }
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            adc_address: 0x48,   // ADS1015/ADS1115 default I2C address
            divider_ratio: 11.0, // 100k/10k voltage divider
            full_voltage: 12.6,  // 3S LiPo full (4.2V * 3)
            empty_voltage: 9.6,  // 3S LiPo empty (3.2V * 3)
        }
    }
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            control_rate: 20.0,          // 20 Hz control loop
            heartbeat_rate: 1.0,         // 1 Hz heartbeat
            odom_rate: 50.0,             // 50 Hz odometry
            cmd_timeout: 0.5,            // 500ms timeout
            battery_check_interval: 2.0, // Check battery every 2s during mission
        }
    }
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
            k_att: 1.0,                  // Attractive gain
            k_rep: 0.8,                  // Repulsive gain
            d_influence: 1.5,            // Start repelling at 1.5m
            d_safe: 0.25,                // Emergency stop at 25cm
            goal_tolerance: 0.1,         // 10cm goal tolerance
            max_linear: 0.3,             // 0.3 m/s max forward speed
            max_angular: 1.5,            // 1.5 rad/s max turn rate
            k_heading: 2.0,              // Heading correction gain
            low_battery_threshold: 15.0, // Return to dock at 15%
            mission_timeout_factor: 5.0, // 500% safety margin on estimated time
        }
    }
}

// ------------------------
// --- Implementation ---
// ------------------------

impl RoverConfig {
    /// Load configuration from file, or return default if not found.
    pub fn load_or_default() -> Result<Self> {
        let config_paths = [
            "/etc/coven/rover.toml",
            "~/.config/coven/rover.toml",
            "./rover.toml",
        ];

        for path in &config_paths {
            let expanded = shellexpand::tilde(path);
            if Path::new(expanded.as_ref()).exists() {
                return Self::load_from_file(&expanded);
            }
        }

        Ok(Self::default())
    }

    /// Validate GPIO pin assignments and check for conflicts.
    pub fn validate_gpio_pins(&self) -> Vec<String> {
        let mut issues = Vec::new();
        let mut pin_usage: std::collections::HashMap<u8, Vec<&str>> =
            std::collections::HashMap::new();

        // Collect all GPIO pin assignments
        let pins = [
            (self.hardware.motors.front_left_pwm, "Motor FL PWM"),
            (self.hardware.motors.front_left_in1, "Motor FL IN1"),
            (self.hardware.motors.front_left_in2, "Motor FL IN2"),
            (self.hardware.motors.front_right_pwm, "Motor FR PWM"),
            (self.hardware.motors.front_right_in1, "Motor FR IN1"),
            (self.hardware.motors.front_right_in2, "Motor FR IN2"),
            (self.hardware.motors.standby_1, "Motor Standby 1"),
            (self.hardware.motors.rear_left_pwm, "Motor RL PWM"),
            (self.hardware.motors.rear_left_in1, "Motor RL IN1"),
            (self.hardware.motors.rear_left_in2, "Motor RL IN2"),
            (self.hardware.motors.rear_right_pwm, "Motor RR PWM"),
            (self.hardware.motors.rear_right_in1, "Motor RR IN1"),
            (self.hardware.motors.rear_right_in2, "Motor RR IN2"),
            (self.hardware.motors.standby_2, "Motor Standby 2"),
            (self.hardware.encoders.front_left_a, "Encoder FL A"),
            (self.hardware.encoders.front_left_b, "Encoder FL B"),
            (self.hardware.encoders.front_right_a, "Encoder FR A"),
            (self.hardware.encoders.front_right_b, "Encoder FR B"),
            (self.hardware.encoders.rear_left_a, "Encoder RL A"),
            (self.hardware.encoders.rear_left_b, "Encoder RL B"),
            (self.hardware.encoders.rear_right_a, "Encoder RR A"),
            (self.hardware.encoders.rear_right_b, "Encoder RR B"),
        ];

        // Build usage map
        for (pin, name) in pins {
            pin_usage.entry(pin).or_default().push(name);
        }

        // Check for conflicts
        for (pin, users) in &pin_usage {
            if users.len() > 1 {
                issues.push(format!(
                    "GPIO{} conflict: used by {}",
                    pin,
                    users.join(", ")
                ));
            }
        }

        // Check for reserved/special pins
        let reserved_pins = [
            (0, "ID_SD (I2C EEPROM)"),
            (1, "ID_SC (I2C EEPROM)"),
            (2, "I2C SDA"),
            (3, "I2C SCL"),
            (14, "UART TX"),
            (15, "UART RX"),
        ];

        for (pin, users) in &pin_usage {
            for (reserved_pin, purpose) in &reserved_pins {
                if pin == reserved_pin {
                    issues.push(format!(
                        "GPIO{} is reserved for {} but assigned to {}",
                        pin,
                        purpose,
                        users.join(", ")
                    ));
                }
            }
        }

        // Check front PWM pins are on hardware PWM channels.
        // Rear PWM pins use pigpio DMA PWM and can be any GPIO.
        if self.hardware.motors.front_left_pwm != 12 && self.hardware.motors.front_left_pwm != 18 {
            issues.push(format!(
                "Front-left PWM pin GPIO{} is not a hardware PWM pin (must be 12 or 18 for PWM0)",
                self.hardware.motors.front_left_pwm
            ));
        }
        if self.hardware.motors.front_right_pwm != 13 && self.hardware.motors.front_right_pwm != 19 {
            issues.push(format!(
                "Front-right PWM pin GPIO{} is not a hardware PWM pin (must be 13 or 19 for PWM1)",
                self.hardware.motors.front_right_pwm
            ));
        }

        // Sanity checks on motor/encoder values
        if self.hardware.motors.wheel_radius <= 0.0 {
            issues.push("Motor wheel_radius must be positive".to_string());
        }
        if self.hardware.motors.wheel_base <= 0.0 {
            issues.push("Motor wheel_base must be positive".to_string());
        }
        if self.hardware.motors.max_rpm <= 0.0 {
            issues.push("Motor max_rpm must be positive".to_string());
        }
        if self.hardware.encoders.pulses_per_rev == 0 {
            issues.push("Encoder pulses_per_rev must be non-zero".to_string());
        }

        // Timing validation (prevent division by zero)
        if self.timing.control_rate <= 0.0 {
            issues.push("timing.control_rate must be positive (e.g., 20.0)".to_string());
        }
        if self.timing.heartbeat_rate <= 0.0 {
            issues.push("timing.heartbeat_rate must be positive (e.g., 1.0)".to_string());
        }
        if self.timing.odom_rate <= 0.0 {
            issues.push("timing.odom_rate must be positive (e.g., 50.0)".to_string());
        }
        if self.timing.cmd_timeout <= 0.0 {
            issues.push("timing.cmd_timeout must be positive (e.g., 0.5)".to_string());
        }

        // PWM frequency validation (TB6612FNG: 100Hz - 100kHz)
        if self.hardware.motors.pwm_frequency < 100.0 || self.hardware.motors.pwm_frequency > 100_000.0 {
            issues.push(format!(
                "Motor pwm_frequency {} Hz out of valid range (100-100000 Hz)",
                self.hardware.motors.pwm_frequency
            ));
        }

        // Navigation parameter validation
        if self.navigation.d_influence <= 0.0 {
            issues.push("navigation.d_influence must be positive".to_string());
        }
        if self.navigation.d_safe <= 0.0 {
            issues.push("navigation.d_safe must be positive".to_string());
        }
        if self.navigation.d_safe >= self.navigation.d_influence {
            issues.push("navigation.d_safe must be less than d_influence".to_string());
        }
        if self.navigation.max_linear <= 0.0 {
            issues.push("navigation.max_linear must be positive".to_string());
        }
        if self.navigation.max_angular <= 0.0 {
            issues.push("navigation.max_angular must be positive".to_string());
        }

        // LiDAR range validation
        if self.hardware.lidar.range_min >= self.hardware.lidar.range_max {
            issues.push("lidar.range_min must be less than range_max".to_string());
        }

        // Battery voltage validation
        if self.hardware.battery.empty_voltage >= self.hardware.battery.full_voltage {
            issues.push("battery.empty_voltage must be less than full_voltage".to_string());
        }

        // Dock UART validation
        // NOTE: COVEN rovers use UART for dock communication, NOT WiFi/Ethernet
        if self.dock_uart.port.is_empty() {
            issues.push("dock_uart.port must be specified (e.g., /dev/ttyAMA0)".to_string());
        }
        if self.dock_uart.baud_rate == 0 {
            issues.push("dock_uart.baud_rate must be non-zero".to_string());
        }
        // Common baud rates for UART
        let valid_baud_rates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];
        if !valid_baud_rates.contains(&self.dock_uart.baud_rate) {
            issues.push(format!(
                "dock_uart.baud_rate {} is non-standard (common: 115200)",
                self.dock_uart.baud_rate
            ));
        }

        // Low battery threshold validation
        if self.navigation.low_battery_threshold <= 0.0 || self.navigation.low_battery_threshold > 100.0 {
            issues.push("navigation.low_battery_threshold must be between 0 and 100".to_string());
        }

        issues
    }

    /// Load configuration from a specific file.
    pub fn load_from_file(path: &str) -> Result<Self> {
        let contents = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read config file: {}", path))?;

        toml::from_str(&contents).with_context(|| format!("Failed to parse config file: {}", path))
    }

    /// Save configuration to file.
    #[allow(dead_code)]
    pub fn save_to_file(&self, path: &str) -> Result<()> {
        let contents = toml::to_string_pretty(self).context("Failed to serialize config")?;

        std::fs::write(path, contents)
            .with_context(|| format!("Failed to write config file: {}", path))
    }
}
