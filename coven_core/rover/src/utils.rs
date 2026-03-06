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
use std::time::{SystemTime, UNIX_EPOCH};

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
