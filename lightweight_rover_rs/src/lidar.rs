//! lidar.rs — YDLiDAR X4 Driver
//!
//! Communicates with the YDLiDAR X4 over serial to receive laser scans.
//! Protocol based on YDLiDAR X4 datasheet.
//!
//! Responsibilities:
//! - Open and configure serial port for LiDAR communication
//! - Send start/stop scan commands
//! - Parse binary scan packets from LiDAR
//! - Assemble complete 360° scans from partial packets
//! - Provide async channel for scan delivery
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::time::Duration;

// --- Third-party ---
use anyhow::{Context, Result};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc;
use tokio_serial::SerialPortBuilderExt;
use tracing::{debug, error, info, trace, warn};

// --- Local ---
use crate::config::LidarConfig;
use crate::utils::now_secs;

// ------------------------
// --- Data Structures ---
// ------------------------

/// A single laser scan (ROS-compatible format).
#[derive(Debug, Clone)]
#[allow(dead_code)] // Some fields are for protocol completeness
pub struct LaserScan {
    /// Timestamp of scan start (Unix seconds).
    pub timestamp: f64,
    /// Minimum angle in radians.
    pub angle_min: f64,
    /// Maximum angle in radians.
    pub angle_max: f64,
    /// Angle increment between measurements (radians).
    pub angle_increment: f64,
    /// Time between measurements (seconds).
    pub time_increment: f64,
    /// Time between scans (seconds).
    pub scan_time: f64,
    /// Minimum range value (meters).
    pub range_min: f64,
    /// Maximum range value (meters).
    pub range_max: f64,
    /// Range measurements (meters), inf for no return.
    pub ranges: Vec<f64>,
    /// Intensity measurements (optional).
    pub intensities: Vec<f64>,
}

impl Default for LaserScan {
    fn default() -> Self {
        Self {
            timestamp: 0.0,
            angle_min: -std::f64::consts::PI,
            angle_max: std::f64::consts::PI,
            angle_increment: 0.0,
            time_increment: 0.0,
            scan_time: 0.0,
            range_min: 0.12,
            range_max: 10.0,
            ranges: Vec::new(),
            intensities: Vec::new(),
        }
    }
}

impl LaserScan {
    /// Convert ranges to millimeters for raw storage.
    pub fn to_ranges_mm(&self) -> Vec<u16> {
        self.ranges
            .iter()
            .map(|r| {
                if r.is_finite() && *r > 0.0 {
                    (r * 1000.0).min(65535.0) as u16
                } else {
                    0 // 0 = invalid/no return
                }
            })
            .collect()
    }

    /// Get number of rays.
    #[allow(dead_code)]
    pub fn num_rays(&self) -> usize {
        self.ranges.len()
    }
}

// ------------------------
// --- Protocol Constants ---
// ------------------------

/// YDLiDAR X4 protocol constants.
mod protocol {
    /// Protocol header byte.
    pub const HEADER: u8 = 0xA5;
    /// Start scanning command.
    pub const CMD_SCAN: u8 = 0x60;
    /// Stop scanning command.
    pub const CMD_STOP: u8 = 0x65;
    /// Scan packet header byte 1.
    pub const SCAN_HEADER_1: u8 = 0xAA;
    /// Scan packet header byte 2.
    pub const SCAN_HEADER_2: u8 = 0x55;
}

/// YDLiDAR driver for async scan acquisition.
pub struct LidarDriver {
    /// LiDAR configuration.
    config: LidarConfig,
    /// Receiver for incoming scans.
    scan_rx: Option<mpsc::Receiver<LaserScan>>,
    /// Sender for stop signal.
    stop_tx: Option<mpsc::Sender<()>>,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl LidarDriver {
    /// Create a new LiDAR driver.
    pub fn new(config: &LidarConfig) -> Self {
        Self {
            config: config.clone(),
            scan_rx: None,
            stop_tx: None,
        }
    }

    /// Start the LiDAR and spawn scan task.
    pub async fn start(&mut self) -> Result<()> {
        let port_name = self.config.port.clone();
        let baud_rate = self.config.baud_rate;
        let range_min = self.config.range_min;
        let range_max = self.config.range_max;

        info!("--- LiDAR Driver ---");
        info!("  Serial port: {}", port_name);
        info!("  Baud rate: {}", baud_rate);
        info!("  Range: {:.2}m - {:.2}m", range_min, range_max);

        // Create channels
        let (scan_tx, scan_rx) = mpsc::channel::<LaserScan>(10);
        let (stop_tx, stop_rx) = mpsc::channel::<()>(1);

        self.scan_rx = Some(scan_rx);
        self.stop_tx = Some(stop_tx);

        // Spawn reader task
        let port_name_clone = port_name.clone();
        tokio::spawn(async move {
            if let Err(e) = Self::scan_loop(
                &port_name_clone,
                baud_rate,
                range_min,
                range_max,
                scan_tx,
                stop_rx,
            )
            .await
            {
                error!("!!! LIDAR SCAN LOOP CRASHED !!!");
                error!("  Error: {:#}", e);
                error!("  Port: {}", port_name_clone);
                error!("  Troubleshooting:");
                error!("    1. Check LiDAR is connected via USB");
                error!("    2. Check udev rule creates {} symlink", port_name_clone);
                error!("    3. Check permissions: ls -la {}", port_name_clone);
                error!("    4. Try: sudo chmod 666 {}", port_name_clone);
            }
        });

        info!("  [OK] LiDAR driver started (scans arriving async)");

        Ok(())
    }

    /// Main scan loop.
    async fn scan_loop(
        port_name: &str,
        baud_rate: u32,
        range_min: f64,
        range_max: f64,
        scan_tx: mpsc::Sender<LaserScan>,
        mut stop_rx: mpsc::Receiver<()>,
    ) -> Result<()> {
        // Open serial port
        debug!("Opening serial port {} at {} baud...", port_name, baud_rate);
        let mut port = tokio_serial::new(port_name, baud_rate)
            .timeout(Duration::from_millis(100))
            .open_native_async()
            .map_err(|e| {
                error!("Cannot open LiDAR serial port {}: {}", port_name, e);
                error!("  Check: Device exists with 'ls {}'", port_name);
                error!("  Check: Permissions with 'ls -la {}'", port_name);
                error!("  If using udev symlink, check rule is active");
                e
            })
            .context(format!("Failed to open serial port: {}", port_name))?;
        info!("LiDAR serial port opened: {}", port_name);

        // Send start scan command
        debug!("Sending LiDAR start command...");
        let start_cmd = [protocol::HEADER, protocol::CMD_SCAN];
        port.write_all(&start_cmd)
            .await
            .map_err(|e| {
                error!("Failed to send start command to LiDAR: {}", e);
                e
            })
            .context("Failed to send start command")?;
        debug!("LiDAR start command sent, waiting for scans...");

        let mut buffer = vec![0u8; 4096];
        let mut partial_scan = PartialScan::new();

        // Track scan statistics for logging
        let mut scan_count: u64 = 0;
        let mut last_scan_log = std::time::Instant::now();
        let mut consecutive_errors: u32 = 0;
        const MAX_CONSECUTIVE_ERRORS: u32 = 50;

        loop {
            tokio::select! {
                _ = stop_rx.recv() => {
                    // Stop command received
                    info!("LiDAR stop command received, shutting down...");
                    let stop_cmd = [protocol::HEADER, protocol::CMD_STOP];
                    let _ = port.write_all(&stop_cmd).await;
                    info!("LiDAR stopped after {} scans", scan_count);
                    break;
                }
                result = port.read(&mut buffer) => {
                    match result {
                        Ok(n) if n > 0 => {
                            consecutive_errors = 0; // Reset error counter
                            trace!("LiDAR received {} bytes", n);

                            // Process received data
                            if let Some(scan) = partial_scan.process(&buffer[..n], range_min, range_max) {
                                scan_count += 1;

                                // Log first scan and then periodically
                                if scan_count == 1 {
                                    info!(
                                        "  [OK] First LiDAR scan received: {} rays, {:.1}° to {:.1}°",
                                        scan.ranges.len(),
                                        scan.angle_min.to_degrees(),
                                        scan.angle_max.to_degrees()
                                    );
                                } else if last_scan_log.elapsed() >= Duration::from_secs(30) {
                                    debug!(
                                        "LiDAR status: {} scans, latest {} rays",
                                        scan_count, scan.ranges.len()
                                    );
                                    last_scan_log = std::time::Instant::now();
                                }

                                if scan_tx.send(scan).await.is_err() {
                                    // Receiver dropped
                                    warn!("LiDAR scan receiver dropped, stopping");
                                    break;
                                }
                            }
                        }
                        Ok(_) => {
                            // Timeout or no data - this is normal
                            trace!("LiDAR read timeout (normal)");
                            tokio::time::sleep(Duration::from_millis(10)).await;
                        }
                        Err(e) => {
                            consecutive_errors += 1;
                            warn!(
                                "LiDAR serial read error ({}/{}): {}",
                                consecutive_errors, MAX_CONSECUTIVE_ERRORS, e
                            );

                            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                                error!("!!! TOO MANY LIDAR ERRORS - GIVING UP !!!");
                                error!("  {} consecutive read errors", consecutive_errors);
                                error!("  Last error: {}", e);
                                error!("  LiDAR may be disconnected or malfunctioning");
                                return Err(anyhow::anyhow!(
                                    "LiDAR communication failed after {} errors: {}",
                                    consecutive_errors, e
                                ));
                            }

                            tokio::time::sleep(Duration::from_millis(100)).await;
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Get the next scan (non-blocking).
    pub async fn get_scan(&mut self) -> Option<LaserScan> {
        if let Some(rx) = &mut self.scan_rx {
            // Use try_recv for non-blocking
            rx.try_recv().ok()
        } else {
            None
        }
    }

    /// Stop the LiDAR.
    pub async fn stop(&mut self) {
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.send(()).await;
        }
    }
}

// ------------------------
// --- Scan Assembly ---
// ------------------------

/// Helper for assembling partial scan packets.
struct PartialScan {
    /// Incoming byte buffer.
    buffer: Vec<u8>,
    /// Current scan points (angle, distance).
    current_scan: Vec<(f64, f64)>,
    /// Last angle seen (for wrap detection).
    last_angle: f64,
    /// Whether a scan has started.
    scan_started: bool,
}

impl PartialScan {
    /// Create a new partial scan assembler.
    fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(4096),
            current_scan: Vec::with_capacity(360),
            last_angle: 0.0,
            scan_started: false,
        }
    }

    /// Process incoming bytes, return complete scan if available.
    fn process(&mut self, data: &[u8], range_min: f64, range_max: f64) -> Option<LaserScan> {
        self.buffer.extend_from_slice(data);

        let mut result = None;

        // Look for scan packets
        while let Some(packet_start) = self.find_packet_header() {
            if packet_start > 0 {
                // Discard bytes before header
                self.buffer.drain(..packet_start);
            }

            // Need at least 10 bytes for header + checksum
            if self.buffer.len() < 10 {
                break;
            }

            // Parse packet header
            let _ct = self.buffer[2];
            let lsn = self.buffer[3] as usize;
            let fsa = u16::from_le_bytes([self.buffer[4], self.buffer[5]]);
            let lsa = u16::from_le_bytes([self.buffer[6], self.buffer[7]]);

            // Calculate packet size: header(10) + samples(lsn * 2)
            let packet_size = 10 + lsn * 2;

            if self.buffer.len() < packet_size {
                // Need more data
                break;
            }

            // Extract distance samples
            let first_angle = Self::decode_angle(fsa);
            let last_angle = Self::decode_angle(lsa);

            // Check for scan boundary (angle wrap)
            if self.scan_started && first_angle < self.last_angle - 90.0 {
                // Complete scan
                result = Some(self.build_scan(range_min, range_max));
                self.current_scan.clear();
            }

            self.scan_started = true;

            // Calculate angle step
            let angle_step = if lsn > 1 {
                (last_angle - first_angle) / (lsn - 1) as f64
            } else {
                0.0
            };

            // Parse distance samples
            for i in 0..lsn {
                let idx = 10 + i * 2;
                let dist_raw = u16::from_le_bytes([self.buffer[idx], self.buffer[idx + 1]]);
                let distance = dist_raw as f64 / 1000.0; // Convert mm to m
                let angle = first_angle + angle_step * i as f64;

                self.current_scan.push((angle, distance));
                self.last_angle = angle;
            }

            // Remove processed packet
            self.buffer.drain(..packet_size);
        }

        result
    }

    /// Find packet header (0xAA, 0x55).
    fn find_packet_header(&self) -> Option<usize> {
        self.buffer
            .windows(2)
            .position(|w| w[0] == protocol::SCAN_HEADER_1 && w[1] == protocol::SCAN_HEADER_2)
    }

    /// Decode angle from raw value.
    fn decode_angle(raw: u16) -> f64 {
        // Angle = (raw >> 1) / 64.0
        ((raw >> 1) as f64) / 64.0
    }

    /// Build a LaserScan from accumulated samples.
    fn build_scan(&self, range_min: f64, range_max: f64) -> LaserScan {
        // Sort by angle
        let mut points: Vec<_> = self.current_scan.clone();
        points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        // Convert to standard format
        let num_points = points.len();
        let angle_min_deg = points.first().map(|(a, _)| *a).unwrap_or(0.0);
        let angle_max_deg = points.last().map(|(a, _)| *a).unwrap_or(360.0);

        let angle_min = angle_min_deg.to_radians() - std::f64::consts::PI;
        let angle_max = angle_max_deg.to_radians() - std::f64::consts::PI;
        let angle_increment = if num_points > 1 {
            (angle_max - angle_min) / (num_points - 1) as f64
        } else {
            0.0
        };

        let ranges: Vec<f64> = points
            .iter()
            .map(|(_, d)| {
                if *d < range_min || *d > range_max {
                    f64::INFINITY
                } else {
                    *d
                }
            })
            .collect();

        LaserScan {
            timestamp: now_secs(),
            angle_min,
            angle_max,
            angle_increment,
            time_increment: 0.0,
            scan_time: 1.0 / 6.0, // ~6 Hz
            range_min,
            range_max,
            ranges,
            intensities: Vec::new(),
        }
    }
}

impl Drop for LidarDriver {
    fn drop(&mut self) {
        // Attempt to stop the scanner
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.try_send(());
        }
    }
}
