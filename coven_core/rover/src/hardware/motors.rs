// SPDX-License-Identifier: MIT
//! motors.rs — TB6612FNG Motor Driver
//!
//! Controls a differential drive robot using PWM for speed and GPIO for direction.
//!
//! Responsibilities:
//! - Initialize PWM channels for motor speed control
//! - Initialize GPIO pins for motor direction
//! - Convert velocity commands to motor speeds
//! - Implement differential drive kinematics
//! - Provide enable/disable and emergency stop
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Third-party ---
use anyhow::{Context, Result};
use rppal::gpio::{Gpio, OutputPin};
use rppal::pwm::{Channel, Polarity, Pwm};
use tracing::{debug, info, trace, warn};

// --- Local ---
use super::init_output_pin;
use crate::config::MotorConfig;
use crate::diagnostics::{log_init_failure, HardwareComponent};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Motor direction.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
enum Direction {
    /// Forward rotation.
    Forward,
    /// Backward rotation.
    Backward,
    /// Active brake (both pins high).
    Brake,
    /// Coast to stop (both pins low).
    Coast,
}

impl Direction {
    /// Determine direction from a signed value (positive = forward).
    #[inline]
    fn from_value(value: f64) -> Self {
        if value >= 0.0 {
            Direction::Forward
        } else {
            Direction::Backward
        }
    }
}

/// Single motor control.
struct Motor {
    /// PWM for speed control.
    pwm: Pwm,
    /// Direction pin 1.
    in1: OutputPin,
    /// Direction pin 2.
    in2: OutputPin,
}

impl Motor {
    /// Set motor direction and duty cycle.
    fn set(&mut self, direction: Direction, duty: f64) {
        // Clamp duty cycle to valid range
        let duty = duty.clamp(0.0, 1.0);

        match direction {
            Direction::Forward => {
                self.in1.set_high();
                self.in2.set_low();
            }
            Direction::Backward => {
                self.in1.set_low();
                self.in2.set_high();
            }
            Direction::Brake => {
                self.in1.set_high();
                self.in2.set_high();
            }
            Direction::Coast => {
                self.in1.set_low();
                self.in2.set_low();
            }
        }

        if let Err(e) = self.pwm.set_duty_cycle(duty) {
            warn!("Failed to set PWM duty cycle: {}", e);
        }
    }

    /// Stop the motor.
    fn stop(&mut self) {
        self.set(Direction::Brake, 0.0);
    }
}

/// Differential drive motor controller.
pub struct MotorController {
    /// Left motor.
    left: Motor,
    /// Right motor.
    right: Motor,
    /// Standby pin (HIGH to enable).
    standby: OutputPin,
    /// Wheel base in meters.
    wheel_base: f64,
    /// Wheel radius in meters.
    wheel_radius: f64,
    /// Maximum motor RPM.
    max_rpm: f64,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl MotorController {
    /// Create a new motor controller with the given configuration.
    pub fn new(config: &MotorConfig) -> Result<Self> {
        trace!("Initializing GPIO subsystem...");
        let gpio = Gpio::new()
            .inspect_err(|e| {
                log_init_failure(HardwareComponent::Gpio, e);
            })
            .context("Failed to initialize GPIO - check permissions and kernel modules")?;
        trace!("GPIO subsystem OK");

        // Initialize standby pin (must be HIGH for motors to run)
        trace!("Configuring standby pin GPIO{}...", config.standby);
        let mut standby = init_output_pin(&gpio, config.standby, "standby")?;
        standby.set_low(); // Start in standby
        trace!("Standby pin configured (LOW = motors disabled)");

        // Initialize left motor PWM
        trace!(
            "Initializing left motor PWM on channel 0 (GPIO{})...",
            config.left_pwm
        );
        let left_pwm = Pwm::with_frequency(
            Channel::Pwm0, // GPIO 12
            config.pwm_frequency,
            0.0,
            Polarity::Normal,
            true,
        )
        .inspect_err(|e| {
            log_init_failure(HardwareComponent::Pwm, e);
        })
        .context("Failed to initialize left PWM (Channel 0 / GPIO12)")?;
        trace!("Left PWM initialized at {} Hz", config.pwm_frequency);

        // Initialize left motor direction pins
        trace!(
            "Configuring left motor direction pins GPIO{}/GPIO{}...",
            config.left_in1,
            config.left_in2
        );
        let left_in1 = init_output_pin(&gpio, config.left_in1, "left IN1")?;
        let left_in2 = init_output_pin(&gpio, config.left_in2, "left IN2")?;
        trace!("Left motor direction pins configured");

        let left = Motor {
            pwm: left_pwm,
            in1: left_in1,
            in2: left_in2,
        };

        // Initialize right motor PWM
        trace!(
            "Initializing right motor PWM on channel 1 (GPIO{})...",
            config.right_pwm
        );
        let right_pwm = Pwm::with_frequency(
            Channel::Pwm1, // GPIO 13
            config.pwm_frequency,
            0.0,
            Polarity::Normal,
            true,
        )
        .inspect_err(|e| {
            log_init_failure(HardwareComponent::Pwm, e);
        })
        .context("Failed to initialize right PWM (Channel 1 / GPIO13)")?;
        trace!("Right PWM initialized at {} Hz", config.pwm_frequency);

        // Initialize right motor direction pins
        trace!(
            "Configuring right motor direction pins GPIO{}/GPIO{}...",
            config.right_in1,
            config.right_in2
        );
        let right_in1 = init_output_pin(&gpio, config.right_in1, "right IN1")?;
        let right_in2 = init_output_pin(&gpio, config.right_in2, "right IN2")?;
        trace!("Right motor direction pins configured");

        let right = Motor {
            pwm: right_pwm,
            in1: right_in1,
            in2: right_in2,
        };

        let mut controller = Self {
            left,
            right,
            standby,
            wheel_base: config.wheel_base,
            wheel_radius: config.wheel_radius,
            max_rpm: config.max_rpm,
        };

        // Enable motors
        controller.enable();
        info!(
            "Motor controller ready: wheel_base={}mm, wheel_radius={}mm, max_rpm={}",
            (config.wheel_base * 1000.0) as u32,
            (config.wheel_radius * 1000.0) as u32,
            config.max_rpm
        );

        Ok(controller)
    }

    /// Enable motor driver (exit standby).
    pub fn enable(&mut self) {
        self.standby.set_high();
        debug!("Motors enabled");
    }

    /// Disable motor driver (enter standby).
    pub fn disable(&mut self) {
        self.standby.set_low();
        debug!("Motors disabled");
    }

    /// Set velocity using linear and angular components (twist-style).
    pub fn set_velocity(&mut self, linear: f64, angular: f64) {
        // Differential drive kinematics
        // v_left = linear - (angular * wheel_base / 2)
        // v_right = linear + (angular * wheel_base / 2)
        let v_left = linear - (angular * self.wheel_base / 2.0);
        let v_right = linear + (angular * self.wheel_base / 2.0);

        // Convert m/s to RPM
        // RPM = (v / (2 * pi * r)) * 60
        let rpm_left = (v_left / (2.0 * std::f64::consts::PI * self.wheel_radius)) * 60.0;
        let rpm_right = (v_right / (2.0 * std::f64::consts::PI * self.wheel_radius)) * 60.0;

        // Convert to duty cycle (0.0 - 1.0)
        let duty_left = (rpm_left.abs() / self.max_rpm).clamp(0.0, 1.0);
        let duty_right = (rpm_right.abs() / self.max_rpm).clamp(0.0, 1.0);

        // Determine direction
        let dir_left = Direction::from_value(rpm_left);
        let dir_right = Direction::from_value(rpm_right);

        // Apply to motors
        self.left.set(dir_left, duty_left);
        self.right.set(dir_right, duty_right);

        debug!(
            linear = linear,
            angular = angular,
            duty_left = duty_left,
            duty_right = duty_right,
            "Velocity set"
        );
    }

    /// Stop both motors immediately.
    pub fn stop(&mut self) {
        self.left.stop();
        self.right.stop();
        debug!("Motors stopped");
    }

    /// Set individual motor speeds directly (for testing/calibration).
    pub fn set_raw(&mut self, left: f64, right: f64) {
        self.left
            .set(Direction::from_value(left), left.abs().clamp(0.0, 1.0));
        self.right
            .set(Direction::from_value(right), right.abs().clamp(0.0, 1.0));
    }
}

impl Drop for MotorController {
    fn drop(&mut self) {
        self.stop();
        self.disable();
    }
}
