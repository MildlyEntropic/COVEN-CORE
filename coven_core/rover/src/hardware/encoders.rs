// SPDX-License-Identifier: MIT
//! encoders.rs — Quadrature Encoder Reader
//!
//! Uses GPIO interrupts to count encoder ticks and compute odometry.
//!
//! Responsibilities:
//! - Initialize GPIO pins with pull-up resistors
//! - Register interrupt handlers for quadrature decoding
//! - Accumulate encoder ticks atomically
//! - Compute odometry from wheel encoders
//! - Provide delta ticks for raw data collection
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::sync::Arc;
use std::time::Instant;

// --- Third-party ---
use anyhow::{Context, Result};
use rppal::gpio::{Event, Gpio, InputPin, Level, Trigger};
use tracing::{debug, error, info, trace};

// --- Local ---
use crate::config::{EncoderConfig, MotorConfig};
use crate::diagnostics::{log_gpio_failure, log_init_failure, HardwareComponent};
use crate::utils::{normalize_angle, now_secs};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Odometry state from wheel encoders.
#[derive(Debug, Clone, Default)]
pub struct Odometry {
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
    /// Timestamp in Unix seconds.
    pub timestamp: f64,
}

/// Shared state for encoder B pin level (updated by B pin interrupt).
struct EncoderBState {
    /// Whether B channel is currently high.
    level_high: AtomicBool,
}

/// Single encoder channel.
struct Encoder {
    /// Channel A input pin (kept alive for interrupt).
    _pin_a: InputPin,
    /// Channel B input pin (kept alive for interrupt).
    _pin_b: InputPin,
    /// Shared B channel state for quadrature decoding.
    #[allow(dead_code)]
    b_state: Arc<EncoderBState>,
    /// Accumulated tick count (updated by interrupt).
    ticks: Arc<AtomicI64>,
}

/// Dual encoder reader for differential drive.
pub struct EncoderReader {
    /// Left wheel encoder.
    left: Encoder,
    /// Right wheel encoder.
    right: Encoder,
    /// Wheel radius in meters.
    wheel_radius: f64,
    /// Wheel base (track width) in meters.
    wheel_base: f64,
    /// Distance traveled per encoder tick in meters.
    meters_per_tick: f64,
    /// Last recorded left wheel ticks (for odometry).
    last_left_ticks: i64,
    /// Last recorded right wheel ticks (for odometry).
    last_right_ticks: i64,
    /// Last recorded left wheel ticks (for raw delta collection).
    last_delta_left_ticks: i64,
    /// Last recorded right wheel ticks (for raw delta collection).
    last_delta_right_ticks: i64,
    /// Timestamp of last update.
    last_time: Instant,
    /// Current odometry state.
    odometry: Odometry,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl EncoderReader {
    /// Create a new encoder reader with the given configuration.
    ///
    /// Requires both encoder config (pin assignments) and motor config (wheel geometry).
    pub fn new(encoder_config: &EncoderConfig, motor_config: &MotorConfig) -> Result<Self> {
        trace!("Initializing encoder GPIO subsystem...");
        let gpio = Gpio::new()
            .inspect_err(|e| {
                log_init_failure(HardwareComponent::Gpio, e);
            })
            .context("Failed to initialize GPIO for encoders")?;

        // Get wheel geometry from motor config
        let wheel_radius = motor_config.wheel_radius;
        let wheel_base = motor_config.wheel_base;

        // Calculate meters per tick
        // Circumference / pulses_per_rev / 2 (X2 quadrature: both edges on channel A only)
        let wheel_circumference = 2.0 * std::f64::consts::PI * wheel_radius;
        let meters_per_tick = wheel_circumference / (encoder_config.pulses_per_rev as f64 * 2.0);

        info!(
            "Encoder resolution: {} pulses/rev -> {:.4} mm/tick (X2 quadrature)",
            encoder_config.pulses_per_rev,
            meters_per_tick * 1000.0
        );

        // Initialize left encoder
        let left = Self::setup_encoder_channel(
            &gpio,
            "left",
            encoder_config.left_a,
            encoder_config.left_b,
        )?;

        // Initialize right encoder
        let right = Self::setup_encoder_channel(
            &gpio,
            "right",
            encoder_config.right_a,
            encoder_config.right_b,
        )?;

        info!("Encoders initialized - interrupts active on 4 GPIO pins");

        Ok(Self {
            left,
            right,
            wheel_radius,
            wheel_base,
            meters_per_tick,
            last_left_ticks: 0,
            last_right_ticks: 0,
            last_delta_left_ticks: 0,
            last_delta_right_ticks: 0,
            last_time: Instant::now(),
            odometry: Odometry::default(),
        })
    }

    /// Setup an encoder channel with error logging.
    fn setup_encoder_channel(gpio: &Gpio, name: &str, pin_a: u8, pin_b: u8) -> Result<Encoder> {
        trace!(
            "Setting up {} encoder on GPIO{}/GPIO{}...",
            name,
            pin_a,
            pin_b
        );
        let ticks = Arc::new(AtomicI64::new(0));

        Self::setup_encoder(gpio, pin_a, pin_b, ticks)
            .inspect_err(|e| {
                log_init_failure(HardwareComponent::Encoder, e);
            })
            .with_context(|| {
                format!(
                    "Failed to setup {} encoder on GPIO{}/GPIO{}",
                    name, pin_a, pin_b
                )
            })
    }

    /// Setup a single quadrature encoder.
    ///
    /// Uses interrupts on both A and B channels:
    /// - B channel interrupt tracks B level in shared atomic state.
    /// - A channel interrupt uses cached B level for direction decoding.
    fn setup_encoder(gpio: &Gpio, pin_a: u8, pin_b: u8, ticks: Arc<AtomicI64>) -> Result<Encoder> {
        trace!("  Acquiring GPIO{} for encoder channel A...", pin_a);
        let mut input_a = gpio
            .get(pin_a)
            .inspect_err(|e| {
                log_gpio_failure(pin_a, "encoder A", e);
            })
            .with_context(|| format!("Failed to get encoder pin A (GPIO{})", pin_a))?
            .into_input_pullup();

        trace!("  Acquiring GPIO{} for encoder channel B...", pin_b);
        let mut input_b = gpio
            .get(pin_b)
            .inspect_err(|e| {
                log_gpio_failure(pin_b, "encoder B", e);
            })
            .with_context(|| format!("Failed to get encoder pin B (GPIO{})", pin_b))?
            .into_input_pullup();

        // Log initial pin states (useful for debugging wiring)
        let initial_a = input_a.read();
        let initial_b = input_b.read();
        trace!("  Initial pin states: A={:?}, B={:?}", initial_a, initial_b);

        // Shared B level state - B interrupt updates, A interrupt reads
        let b_state = Arc::new(EncoderBState {
            level_high: AtomicBool::new(initial_b == Level::High),
        });

        // Set up interrupt on channel B to track its level
        trace!(
            "  Registering interrupt handler for channel B (GPIO{})...",
            pin_b
        );
        let b_state_for_b = b_state.clone();
        input_b
            .set_async_interrupt(Trigger::Both, None, move |event: Event| {
                // Update cached B level based on which edge triggered
                let is_high = event.trigger == Trigger::RisingEdge;
                b_state_for_b.level_high.store(is_high, Ordering::Relaxed);
            })
            .map_err(|e| {
                error!("Failed to register interrupt on GPIO{}: {}", pin_b, e);
                error!("  Async interrupts may not be supported or pin is busy");
                e
            })
            .context(format!(
                "Failed to set up encoder B interrupt on GPIO{}",
                pin_b
            ))?;

        // Set up interrupt on channel A for quadrature decoding
        trace!(
            "  Registering interrupt handler for channel A (GPIO{})...",
            pin_a
        );
        let ticks_clone = ticks.clone();
        let b_state_for_a = b_state.clone();

        input_a
            .set_async_interrupt(Trigger::Both, None, move |event: Event| {
                // Simple quadrature decoding using cached B level
                let b_high = b_state_for_a.level_high.load(Ordering::Relaxed);
                let a_rising = event.trigger == Trigger::RisingEdge;

                let increment = match (a_rising, b_high) {
                    (true, false) => 1,   // A rising, B low = forward
                    (true, true) => -1,   // A rising, B high = backward
                    (false, true) => 1,   // A falling, B high = forward
                    (false, false) => -1, // A falling, B low = backward
                };
                ticks_clone.fetch_add(increment, Ordering::Relaxed);
            })
            .map_err(|e| {
                error!("Failed to register interrupt on GPIO{}: {}", pin_a, e);
                error!("  Async interrupts may not be supported or pin is busy");
                e
            })
            .context(format!(
                "Failed to set up encoder A interrupt on GPIO{}",
                pin_a
            ))?;

        trace!(
            "  Encoder GPIO{}/GPIO{} ready with interrupts",
            pin_a,
            pin_b
        );

        Ok(Encoder {
            _pin_a: input_a,
            _pin_b: input_b,
            b_state,
            ticks,
        })
    }

    /// Get current tick counts.
    pub fn get_ticks(&self) -> (i64, i64) {
        let left = self.left.ticks.load(Ordering::Relaxed);
        let right = self.right.ticks.load(Ordering::Relaxed);
        (left, right)
    }

    /// Reset tick counts to zero.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.left.ticks.store(0, Ordering::Relaxed);
        self.right.ticks.store(0, Ordering::Relaxed);
        self.last_left_ticks = 0;
        self.last_right_ticks = 0;
        self.last_delta_left_ticks = 0;
        self.last_delta_right_ticks = 0;
        self.odometry = Odometry::default();
    }

    /// Update odometry from encoder ticks.
    ///
    /// Call this at a regular rate (e.g., 50 Hz).
    pub fn update(&mut self) -> Odometry {
        let now = Instant::now();
        let dt = now.duration_since(self.last_time).as_secs_f64();

        if dt < 0.001 {
            // Avoid division by zero
            return self.odometry.clone();
        }

        // Read current ticks
        let (left_ticks, right_ticks) = self.get_ticks();

        // Calculate deltas
        let delta_left = left_ticks - self.last_left_ticks;
        let delta_right = right_ticks - self.last_right_ticks;

        // Convert to distances
        let dist_left = delta_left as f64 * self.meters_per_tick;
        let dist_right = delta_right as f64 * self.meters_per_tick;

        // Calculate linear and angular displacement
        let dist_center = (dist_left + dist_right) / 2.0;
        let delta_theta = (dist_right - dist_left) / self.wheel_base;

        // Update pose using midpoint integration
        let theta_mid = self.odometry.theta + delta_theta / 2.0;
        self.odometry.x += dist_center * theta_mid.cos();
        self.odometry.y += dist_center * theta_mid.sin();
        self.odometry.theta += delta_theta;

        // Normalize theta to [-pi, pi]
        self.odometry.theta = normalize_angle(self.odometry.theta);

        // Calculate velocities
        self.odometry.v_linear = dist_center / dt;
        self.odometry.v_angular = delta_theta / dt;

        // Update timestamp
        self.odometry.timestamp = now_secs();

        // Save state for next iteration
        self.last_left_ticks = left_ticks;
        self.last_right_ticks = right_ticks;
        self.last_time = now;

        debug!(
            x = self.odometry.x,
            y = self.odometry.y,
            theta = self.odometry.theta,
            v = self.odometry.v_linear,
            w = self.odometry.v_angular,
            "Odometry updated"
        );

        self.odometry.clone()
    }

    /// Get current odometry without updating.
    #[allow(dead_code)]
    pub fn get_odometry(&self) -> Odometry {
        self.odometry.clone()
    }

    /// Get delta ticks since last call (for raw data collection).
    ///
    /// Returns (left_delta, right_delta) and resets the delta counters.
    /// Uses separate tracking from `update()` so both can be called
    /// independently without consuming each other's deltas.
    pub fn get_delta_ticks(&mut self) -> (i32, i32) {
        let (left_ticks, right_ticks) = self.get_ticks();

        let delta_left = (left_ticks - self.last_delta_left_ticks) as i32;
        let delta_right = (right_ticks - self.last_delta_right_ticks) as i32;

        self.last_delta_left_ticks = left_ticks;
        self.last_delta_right_ticks = right_ticks;

        (delta_left, delta_right)
    }

    /// Perform sanity check on encoder readings.
    ///
    /// Returns warnings if encoders appear stuck or noisy.
    /// Call this periodically (e.g., every few seconds) during operation.
    #[allow(dead_code)]
    pub fn sanity_check(&self, dt_secs: f64, expected_moving: bool) -> Vec<String> {
        let mut warnings = Vec::new();
        let (left, right) = self.get_ticks();

        // Check for unreasonably high tick rates (noise)
        // JGA25-371: 408 PPR * X2 quadrature = 816 ticks/rev
        // At 100 RPM = 100/60 * 816 = ~1360 ticks/sec per wheel
        // Allow 2x margin = ~2750 ticks/sec
        let max_reasonable_ticks_per_sec = 2750.0;
        let max_delta = (max_reasonable_ticks_per_sec * dt_secs) as i64;

        let delta_left = (left - self.last_left_ticks).abs();
        let delta_right = (right - self.last_right_ticks).abs();

        if delta_left > max_delta {
            let rate = delta_left as f64 / dt_secs;
            warnings.push(format!(
                "Left encoder: {} ticks in {:.2}s ({:.0} ticks/sec) - possible noise/bounce",
                delta_left, dt_secs, rate
            ));
        }

        if delta_right > max_delta {
            let rate = delta_right as f64 / dt_secs;
            warnings.push(format!(
                "Right encoder: {} ticks in {:.2}s ({:.0} ticks/sec) - possible noise/bounce",
                delta_right, dt_secs, rate
            ));
        }

        // Check for stuck encoders when we expect movement
        if expected_moving && dt_secs > 0.5 {
            if delta_left == 0 {
                warnings.push(
                    "Left encoder: no ticks detected while moving - check wiring/connection"
                        .to_string(),
                );
            }
            if delta_right == 0 {
                warnings.push(
                    "Right encoder: no ticks detected while moving - check wiring/connection"
                        .to_string(),
                );
            }
        }

        // Check for severe imbalance (one wheel moving, other stuck)
        if delta_left > 10 && delta_right == 0 && expected_moving {
            warnings.push(
                "Right encoder appears stuck while left is counting - check right encoder"
                    .to_string(),
            );
        }
        if delta_right > 10 && delta_left == 0 && expected_moving {
            warnings.push(
                "Left encoder appears stuck while right is counting - check left encoder"
                    .to_string(),
            );
        }

        warnings
    }

    /// Get ticks per revolution (for batch config).
    pub fn ticks_per_rev(&self) -> u16 {
        // circumference / meters_per_tick = ticks per revolution (with 4x quadrature)
        let circumference = 2.0 * std::f64::consts::PI * self.wheel_radius;
        (circumference / self.meters_per_tick) as u16
    }

    /// Get wheel radius in mm.
    pub fn wheel_radius_mm(&self) -> u16 {
        (self.wheel_radius * 1000.0) as u16
    }

    /// Get wheel base in mm.
    pub fn wheel_base_mm(&self) -> u16 {
        (self.wheel_base * 1000.0) as u16
    }
}
