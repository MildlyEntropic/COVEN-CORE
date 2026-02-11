//! utils.rs — Common Utility Functions
//!
//! Shared utility functions used across the rover codebase.
//!
//! Responsibilities:
//! - Provide timestamp acquisition helper
//! - Provide angle normalization for heading calculations
//! - Provide I/O statistics tracking
//! - Reduce code duplication across modules
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::f64::consts::PI;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

// ------------------------
// --- Time Utilities ---
// ------------------------

/// Get current Unix timestamp in seconds as f64.
///
/// Returns 0.0 if system time is before Unix epoch (shouldn't happen).
#[inline]
pub fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

// ------------------------
// --- Angle Utilities ---
// ------------------------

/// Normalize angle to [-π, π] range.
///
/// Used for heading/theta values in odometry and navigation.
/// Guards against infinite loops from NaN/infinity input.
#[inline]
pub fn normalize_angle(angle: f64) -> f64 {
    // Guard against non-finite values to prevent infinite loop
    if !angle.is_finite() {
        return 0.0;
    }

    // Use modulo for efficient single-step normalization
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

// ------------------------
// --- Message Routing ---
// ------------------------

/// Check if a message is addressed to this module.
///
/// Returns true if the message should be processed by this module.
/// A message is for us if:
/// - module_id matches our id exactly, OR
/// - module_id is empty (broadcast)
#[inline]
#[allow(dead_code)]
pub fn is_message_for_us(message_module_id: &str, our_module_id: &str) -> bool {
    message_module_id.is_empty() || message_module_id == our_module_id
}

// ------------------------
// --- I/O Statistics ---
// ------------------------

/// I/O statistics tracker for network and serial communication.
#[derive(Debug, Clone, Default)]
pub struct IoStats {
    /// Messages sent.
    pub messages_sent: u64,
    /// Messages received.
    pub messages_received: u64,
    /// Bytes sent.
    pub bytes_sent: u64,
    /// Bytes received.
    pub bytes_received: u64,
    /// Parse errors encountered.
    pub parse_errors: u64,
    /// Last time stats were logged.
    last_log: Option<Instant>,
}

impl IoStats {
    /// Create a new stats tracker.
    pub fn new() -> Self {
        Self {
            last_log: Some(Instant::now()),
            ..Default::default()
        }
    }

    /// Record a sent message.
    pub fn record_send(&mut self, bytes: usize) {
        self.messages_sent += 1;
        self.bytes_sent += bytes as u64;
    }

    /// Record a received message.
    pub fn record_receive(&mut self, bytes: usize) {
        self.messages_received += 1;
        self.bytes_received += bytes as u64;
    }

    /// Record a parse error.
    pub fn record_error(&mut self) {
        self.parse_errors += 1;
    }

    /// Check if it's time to log stats (returns true every `interval_secs`).
    pub fn should_log(&mut self, interval_secs: f64) -> bool {
        if let Some(last) = self.last_log {
            if last.elapsed().as_secs_f64() >= interval_secs {
                self.last_log = Some(Instant::now());
                return true;
            }
        }
        false
    }

    /// Reset all counters.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.messages_sent = 0;
        self.messages_received = 0;
        self.bytes_sent = 0;
        self.bytes_received = 0;
        self.parse_errors = 0;
        self.last_log = Some(Instant::now());
    }
}

// ------------------------
// --- Tests ---
// ------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_now_secs_returns_positive() {
        let ts = now_secs();
        assert!(ts > 0.0);
    }

    #[test]
    fn test_normalize_angle_already_normalized() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(PI / 2.0) - PI / 2.0).abs() < 1e-10);
        assert!((normalize_angle(-PI / 2.0) - (-PI / 2.0)).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_angle_over_pi() {
        let result = normalize_angle(3.0 * PI / 2.0);
        assert!((result - (-PI / 2.0)).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_angle_under_negative_pi() {
        let result = normalize_angle(-3.0 * PI / 2.0);
        assert!((result - (PI / 2.0)).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_angle_multiple_rotations() {
        let result = normalize_angle(5.0 * PI);
        assert!((result - PI).abs() < 1e-10);
    }
}
