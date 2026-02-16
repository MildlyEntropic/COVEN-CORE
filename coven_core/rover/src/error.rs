//! error.rs — COVEN Rover Error Types
//!
//! Defines typed error variants for the rover daemon.
//!
//! Responsibilities:
//! - Define error types for hardware failures
//! - Define error types for network/protocol issues
//! - Define error types for configuration problems
//!
//! Note: Currently unused as the codebase uses anyhow for error handling.
//! Kept for potential future use with typed error matching.
//!
//! Author: Alexander Shultis
//! Date: January 2026

#![allow(dead_code)]

// ------------------------
// --- Imports ---
// ------------------------

// --- Third-party ---
use thiserror::Error;

// ------------------------
// --- Error Types ---
// ------------------------

/// Error types for rover operations.
#[derive(Error, Debug)]
pub enum RoverError {
    #[error("Hardware initialization failed: {0}")]
    HardwareInit(String),

    #[error("GPIO error: {0}")]
    Gpio(#[from] rppal::gpio::Error),

    #[error("PWM error: {0}")]
    Pwm(#[from] rppal::pwm::Error),

    #[error("Serial port error: {0}")]
    Serial(#[from] tokio_serial::Error),

    #[error("Network error: {0}")]
    Network(#[from] std::io::Error),

    #[error("Protocol error: {0}")]
    Protocol(String),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("LiDAR error: {0}")]
    Lidar(String),

    #[error("Timeout: {0}")]
    Timeout(String),

    #[error("Invalid state transition: {from:?} -> {to:?}")]
    InvalidStateTransition { from: String, to: String },
}
