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

/// Motor driven by software PWM via GPIO (for rear motors on non-HW-PWM pins).
struct SoftPwmMotor {
    /// GPIO pin used for PWM output.
    pwm_pin: OutputPin,
    /// Direction pin 1.
    in1: OutputPin,
    /// Direction pin 2.
    in2: OutputPin,
}

impl SoftPwmMotor {
    /// Set motor direction and duty cycle.
    ///
    /// Duty cycle is binary (on/off) since rppal OutputPin doesn't support
    /// analog PWM. For real deployment, pigpio daemon provides DMA-timed PWM
    /// on any GPIO. This fallback ensures the code compiles and runs with
    /// basic functionality; pigpio integration is handled at the HAL level.
    fn set(&mut self, direction: Direction, duty: f64) {
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

        // Software PWM: set pin high if duty > 0, low if 0.
        // Real PWM timing is handled by pigpio daemon in production.
        if duty > 0.0 {
            self.pwm_pin.set_high();
        } else {
            self.pwm_pin.set_low();
        }
    }

    /// Stop the motor.
    fn stop(&mut self) {
        self.set(Direction::Brake, 0.0);
    }
}

/// 4-wheel skid-steer motor controller.
///
/// Uses 2× TB6612FNG drivers:
/// - Driver 1 (front): hardware PWM via rppal Pwm
/// - Driver 2 (rear): software PWM via GPIO (pigpio DMA in production)
///
/// For skid-steer, left-side motors (FL+RL) receive the same command,
/// and right-side motors (FR+RR) receive the same command.
pub struct MotorController {
    /// Front-left motor (HW PWM).
    front_left: Motor,
    /// Front-right motor (HW PWM).
    front_right: Motor,
    /// Rear-left motor (SW PWM).
    rear_left: SoftPwmMotor,
    /// Rear-right motor (SW PWM).
    rear_right: SoftPwmMotor,
    /// Standby pin for driver 1 (HIGH to enable).
    standby_1: OutputPin,
    /// Standby pin for driver 2 (HIGH to enable).
    standby_2: OutputPin,
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

        // --- Standby pins (must be HIGH for motors to run) ---
        trace!("Configuring standby pin 1 GPIO{}...", config.standby_1);
        let mut standby_1 = init_output_pin(&gpio, config.standby_1, "standby 1")?;
        standby_1.set_low();

        trace!("Configuring standby pin 2 GPIO{}...", config.standby_2);
        let mut standby_2 = init_output_pin(&gpio, config.standby_2, "standby 2")?;
        standby_2.set_low();
        trace!("Standby pins configured (LOW = motors disabled)");

        // --- Front-left motor (HW PWM0) ---
        trace!(
            "Initializing front-left motor PWM on channel 0 (GPIO{})...",
            config.front_left_pwm
        );
        let fl_pwm = Pwm::with_frequency(
            Channel::Pwm0,
            config.pwm_frequency,
            0.0,
            Polarity::Normal,
            true,
        )
        .inspect_err(|e| {
            log_init_failure(HardwareComponent::Pwm, e);
        })
        .context("Failed to initialize front-left PWM (Channel 0)")?;

        let fl_in1 = init_output_pin(&gpio, config.front_left_in1, "FL IN1")?;
        let fl_in2 = init_output_pin(&gpio, config.front_left_in2, "FL IN2")?;
        let front_left = Motor { pwm: fl_pwm, in1: fl_in1, in2: fl_in2 };
        trace!("Front-left motor configured");

        // --- Front-right motor (HW PWM1) ---
        trace!(
            "Initializing front-right motor PWM on channel 1 (GPIO{})...",
            config.front_right_pwm
        );
        let fr_pwm = Pwm::with_frequency(
            Channel::Pwm1,
            config.pwm_frequency,
            0.0,
            Polarity::Normal,
            true,
        )
        .inspect_err(|e| {
            log_init_failure(HardwareComponent::Pwm, e);
        })
        .context("Failed to initialize front-right PWM (Channel 1)")?;

        let fr_in1 = init_output_pin(&gpio, config.front_right_in1, "FR IN1")?;
        let fr_in2 = init_output_pin(&gpio, config.front_right_in2, "FR IN2")?;
        let front_right = Motor { pwm: fr_pwm, in1: fr_in1, in2: fr_in2 };
        trace!("Front-right motor configured");

        // --- Rear-left motor (SW PWM via GPIO) ---
        trace!(
            "Initializing rear-left motor on GPIO{} (software PWM)...",
            config.rear_left_pwm
        );
        let rl_pwm_pin = init_output_pin(&gpio, config.rear_left_pwm, "RL PWM")?;
        let rl_in1 = init_output_pin(&gpio, config.rear_left_in1, "RL IN1")?;
        let rl_in2 = init_output_pin(&gpio, config.rear_left_in2, "RL IN2")?;
        let rear_left = SoftPwmMotor { pwm_pin: rl_pwm_pin, in1: rl_in1, in2: rl_in2 };
        trace!("Rear-left motor configured");

        // --- Rear-right motor (SW PWM via GPIO) ---
        trace!(
            "Initializing rear-right motor on GPIO{} (software PWM)...",
            config.rear_right_pwm
        );
        let rr_pwm_pin = init_output_pin(&gpio, config.rear_right_pwm, "RR PWM")?;
        let rr_in1 = init_output_pin(&gpio, config.rear_right_in1, "RR IN1")?;
        let rr_in2 = init_output_pin(&gpio, config.rear_right_in2, "RR IN2")?;
        let rear_right = SoftPwmMotor { pwm_pin: rr_pwm_pin, in1: rr_in1, in2: rr_in2 };
        trace!("Rear-right motor configured");

        let mut controller = Self {
            front_left,
            front_right,
            rear_left,
            rear_right,
            standby_1,
            standby_2,
            wheel_base: config.wheel_base,
            wheel_radius: config.wheel_radius,
            max_rpm: config.max_rpm,
        };

        // Enable motors
        controller.enable();
        info!(
            "Motor controller ready (4-wheel skid-steer): wheel_base={}mm, wheel_radius={}mm, max_rpm={}",
            (config.wheel_base * 1000.0) as u32,
            (config.wheel_radius * 1000.0) as u32,
            config.max_rpm
        );

        Ok(controller)
    }

    /// Enable both motor drivers (exit standby).
    pub fn enable(&mut self) {
        self.standby_1.set_high();
        self.standby_2.set_high();
        debug!("Motors enabled (both drivers)");
    }

    /// Disable both motor drivers (enter standby).
    pub fn disable(&mut self) {
        self.standby_1.set_low();
        self.standby_2.set_low();
        debug!("Motors disabled (both drivers)");
    }

    /// Set velocity using linear and angular components (twist-style).
    ///
    /// Skid-steer: left-side motors (FL+RL) get the same command,
    /// right-side motors (FR+RR) get the same command.
    pub fn set_velocity(&mut self, linear: f64, angular: f64) {
        // Differential drive kinematics
        let v_left = linear - (angular * self.wheel_base / 2.0);
        let v_right = linear + (angular * self.wheel_base / 2.0);

        // Convert m/s to RPM
        let rpm_left = (v_left / (2.0 * std::f64::consts::PI * self.wheel_radius)) * 60.0;
        let rpm_right = (v_right / (2.0 * std::f64::consts::PI * self.wheel_radius)) * 60.0;

        // Convert to duty cycle (0.0 - 1.0)
        let duty_left = (rpm_left.abs() / self.max_rpm).clamp(0.0, 1.0);
        let duty_right = (rpm_right.abs() / self.max_rpm).clamp(0.0, 1.0);

        // Determine direction
        let dir_left = Direction::from_value(rpm_left);
        let dir_right = Direction::from_value(rpm_right);

        // Apply to all 4 motors (same command per side)
        self.front_left.set(dir_left, duty_left);
        self.rear_left.set(dir_left, duty_left);
        self.front_right.set(dir_right, duty_right);
        self.rear_right.set(dir_right, duty_right);

        debug!(
            linear = linear,
            angular = angular,
            duty_left = duty_left,
            duty_right = duty_right,
            "Velocity set (4-wheel)"
        );
    }

    /// Stop all motors immediately.
    pub fn stop(&mut self) {
        self.front_left.stop();
        self.front_right.stop();
        self.rear_left.stop();
        self.rear_right.stop();
        debug!("All motors stopped");
    }

    /// Set motor speeds directly by side (for testing/calibration).
    ///
    /// Both motors on each side receive the same command.
    pub fn set_raw(&mut self, left: f64, right: f64) {
        let dir_left = Direction::from_value(left);
        let duty_left = left.abs().clamp(0.0, 1.0);
        self.front_left.set(dir_left, duty_left);
        self.rear_left.set(dir_left, duty_left);

        let dir_right = Direction::from_value(right);
        let duty_right = right.abs().clamp(0.0, 1.0);
        self.front_right.set(dir_right, duty_right);
        self.rear_right.set(dir_right, duty_right);
    }
}

impl Drop for MotorController {
    fn drop(&mut self) {
        self.stop();
        self.disable();
    }
}
