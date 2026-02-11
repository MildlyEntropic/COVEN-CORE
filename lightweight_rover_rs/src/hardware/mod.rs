//! mod.rs — COVEN Hardware Abstraction Layer
//!
//! Provides unified access to motors, encoders, battery, and other GPIO peripherals.
//!
//! Responsibilities:
//! - Initialize all hardware components
//! - Provide unified hardware interface
//! - Handle emergency stop and shutdown
//! - Report detailed diagnostic information on failures
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Module Declarations ---
// ------------------------

mod battery;
pub mod encoders;
mod motors;

// ------------------------
// --- Public Exports ---
// ------------------------

pub use battery::BatteryReader;
pub use encoders::EncoderReader;
pub use motors::MotorController;

// ------------------------
// --- Imports ---
// ------------------------

// --- Third-party ---
use anyhow::{Context, Result};
use rppal::gpio::{Gpio, OutputPin};
use tracing::{info, warn};

// --- Local ---
use crate::config::RoverConfig;
use crate::diagnostics::{log_gpio_failure, log_init_failure, HardwareComponent};

// ------------------------
// --- GPIO Helpers ---
// ------------------------

/// Initialize a GPIO pin as output with error logging.
pub fn init_output_pin(gpio: &Gpio, pin: u8, description: &str) -> Result<OutputPin> {
    gpio.get(pin)
        .inspect_err(|e| {
            log_gpio_failure(pin, description, e);
        })
        .with_context(|| format!("Failed to get {} pin GPIO{}", description, pin))
        .map(|p| p.into_output())
}

// ------------------------
// --- Data Structures ---
// ------------------------

/// Combined hardware interface.
pub struct Hardware {
    /// Motor controller for differential drive.
    pub motors: MotorController,
    /// Encoder reader for odometry.
    pub encoders: EncoderReader,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl Hardware {
    /// Initialize all hardware peripherals.
    pub fn new(config: &RoverConfig) -> Result<Self> {
        info!("========== HARDWARE INITIALIZATION ==========");
        info!("Initializing hardware for rover: {}", config.rover_id);

        // --- Motor Controller ---
        info!("--- Motor Controller ---");
        info!(
            "  PWM pins: left=GPIO{} (PWM0), right=GPIO{} (PWM1)",
            config.hardware.motors.left_pwm, config.hardware.motors.right_pwm
        );
        info!(
            "  Direction pins: left=GPIO{}/{}, right=GPIO{}/{}",
            config.hardware.motors.left_in1,
            config.hardware.motors.left_in2,
            config.hardware.motors.right_in1,
            config.hardware.motors.right_in2
        );
        info!("  Standby pin: GPIO{}", config.hardware.motors.standby);
        info!(
            "  PWM frequency: {} Hz",
            config.hardware.motors.pwm_frequency
        );

        let motors = MotorController::new(&config.hardware.motors)
            .inspect_err(|e| {
                log_init_failure(HardwareComponent::Motor, e);
            })
            .context("Failed to initialize motor controller")?;
        info!("  [OK] Motor controller initialized successfully");

        // --- Encoders ---
        info!("--- Encoders ---");
        info!(
            "  Left encoder: A=GPIO{}, B=GPIO{}",
            config.hardware.encoders.left_a, config.hardware.encoders.left_b
        );
        info!(
            "  Right encoder: A=GPIO{}, B=GPIO{}",
            config.hardware.encoders.right_a, config.hardware.encoders.right_b
        );
        info!(
            "  Pulses per rev: {}",
            config.hardware.encoders.pulses_per_rev
        );
        info!(
            "  Wheel geometry: radius={}mm, base={}mm",
            (config.hardware.motors.wheel_radius * 1000.0) as u32,
            (config.hardware.motors.wheel_base * 1000.0) as u32
        );

        let encoders = EncoderReader::new(&config.hardware.encoders, &config.hardware.motors)
            .inspect_err(|e| {
                log_init_failure(HardwareComponent::Encoder, e);
            })
            .context("Failed to initialize encoders")?;
        info!("  [OK] Encoders initialized successfully");

        // --- Summary ---
        info!("========== HARDWARE INIT COMPLETE ==========");
        info!("All hardware components initialized successfully!");
        info!("  Motors: READY");
        info!("  Encoders: READY");
        info!("============================================");

        Ok(Self { motors, encoders })
    }

    /// Emergency stop - cut all motor power.
    #[allow(dead_code)]
    pub fn emergency_stop(&mut self) {
        warn!("!!! EMERGENCY STOP TRIGGERED !!!");
        self.motors.stop();
    }

    /// Clean shutdown of all hardware.
    pub fn shutdown(&mut self) {
        info!("Hardware shutdown requested");
        self.motors.stop();
        info!("Hardware shutdown complete");
        // Encoders don't need explicit shutdown
    }
}

impl Drop for Hardware {
    fn drop(&mut self) {
        self.shutdown();
    }
}
