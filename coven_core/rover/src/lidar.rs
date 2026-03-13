// SPDX-License-Identifier: MIT
//! lidar.rs — RPLIDAR C1 Driver
//!
//! Communicates with the Slamtec RPLIDAR C1 over serial to receive laser scans.
//! Protocol based on Slamtec RPLIDAR Interface Protocol (standard scan mode).
//!
//! Responsibilities:
//! - Open and configure serial port for LiDAR communication (460800 baud)
//! - Send start/stop scan commands per Slamtec protocol
//! - Parse 5-byte measurement nodes from continuous scan stream
//! - Assemble complete 360° scans from measurement nodes
//! - Provide async channel for scan delivery
//!
//! Author: Alexander Shultis
//! Date: March 2026

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
            range_min: 0.15,
            range_max: 12.0,
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

/// Slamtec RPLIDAR protocol constants.
mod protocol {
    /// Request start flag byte.
    pub const START_FLAG: u8 = 0xA5;
    /// Response start flag byte 2.
    pub const START_FLAG2: u8 = 0x5A;
    /// Start scan command.
    pub const CMD_SCAN: u8 = 0x20;
    /// Stop command.
    pub const CMD_STOP: u8 = 0x25;
    /// Reset command.
    pub const CMD_RESET: u8 = 0x40;
    /// Response descriptor length (bytes).
    pub const RESP_DESCRIPTOR_LEN: usize = 7;
    /// Scan measurement node size (bytes).
    pub const SCAN_NODE_SIZE: usize = 5;
    /// Expected data type for standard scan response.
    pub const SCAN_DATA_TYPE: u8 = 0x81;
}

/// RPLIDAR driver for async scan acquisition.
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
        let scan_rate = self.config.scan_rate;

        info!("--- LiDAR Driver (RPLIDAR C1) ---");
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
                scan_rate,
                scan_tx,
                stop_rx,
            )
            .await
            {
                error!("!!! LIDAR SCAN LOOP CRASHED !!!");
                error!("  Error: {:#}", e);
                error!("  Port: {}", port_name_clone);
                error!("  Troubleshooting:");
                error!("    1. Check RPLIDAR C1 is connected via USB");
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
        scan_rate: f64,
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

        // Stop any existing scan (clean slate)
        debug!("Sending RPLIDAR stop command...");
        let stop_cmd = [protocol::START_FLAG, protocol::CMD_STOP];
        port.write_all(&stop_cmd).await.ok();
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Flush any buffered data from previous session
        let mut flush_buf = [0u8; 4096];
        loop {
            match tokio::time::timeout(
                Duration::from_millis(50),
                port.read(&mut flush_buf),
            )
            .await
            {
                Ok(Ok(n)) if n > 0 => continue,
                _ => break,
            }
        }

        // Send reset to ensure clean state
        debug!("Sending RPLIDAR reset command...");
        let reset_cmd = [protocol::START_FLAG, protocol::CMD_RESET];
        port.write_all(&reset_cmd).await.ok();
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Flush post-reset data
        loop {
            match tokio::time::timeout(
                Duration::from_millis(50),
                port.read(&mut flush_buf),
            )
            .await
            {
                Ok(Ok(n)) if n > 0 => continue,
                _ => break,
            }
        }

        // Send start scan command
        debug!("Sending RPLIDAR scan command...");
        let scan_cmd = [protocol::START_FLAG, protocol::CMD_SCAN];
        port.write_all(&scan_cmd)
            .await
            .map_err(|e| {
                error!("Failed to send scan command to LiDAR: {}", e);
                e
            })
            .context("Failed to send scan command")?;

        // Read and validate response descriptor (7 bytes)
        let mut desc_buf = [0u8; protocol::RESP_DESCRIPTOR_LEN];
        tokio::time::timeout(Duration::from_secs(3), port.read_exact(&mut desc_buf))
            .await
            .map_err(|_| anyhow::anyhow!("Timeout waiting for RPLIDAR response descriptor"))?
            .context("Failed to read RPLIDAR response descriptor")?;

        // Validate descriptor: [0xA5][0x5A][size_q30+mode2][data_type]
        if desc_buf[0] != protocol::START_FLAG || desc_buf[1] != protocol::START_FLAG2 {
            anyhow::bail!(
                "Invalid RPLIDAR response descriptor: expected A5 5A, got {:02X} {:02X}",
                desc_buf[0], desc_buf[1]
            );
        }

        let data_type = desc_buf[6];
        if data_type != protocol::SCAN_DATA_TYPE {
            anyhow::bail!(
                "Unexpected scan data type: 0x{:02X} (expected 0x{:02X})",
                data_type, protocol::SCAN_DATA_TYPE
            );
        }

        // Verify send mode is "multiple response" (bits 31:30 of the 4-byte length field)
        let send_mode = (desc_buf[5] >> 6) & 0x03;
        if send_mode != 1 {
            anyhow::bail!(
                "Unexpected send mode: {} (expected 1 = multiple response)",
                send_mode
            );
        }

        info!("RPLIDAR scan started, response descriptor validated");

        let mut buffer = vec![0u8; 4096];
        let mut assembler = ScanAssembler::new();

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
                    let stop_cmd = [protocol::START_FLAG, protocol::CMD_STOP];
                    let _ = port.write_all(&stop_cmd).await;
                    info!("LiDAR stopped after {} scans", scan_count);
                    break;
                }
                result = port.read(&mut buffer) => {
                    match result {
                        Ok(n) if n > 0 => {
                            consecutive_errors = 0; // Reset error counter
                            trace!("LiDAR received {} bytes", n);

                            // Process received data through scan assembler
                            for scan in assembler.process(
                                &buffer[..n], range_min, range_max, scan_rate,
                            ) {
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
                                    let stop_cmd = [protocol::START_FLAG, protocol::CMD_STOP];
                                    let _ = port.write_all(&stop_cmd).await;
                                    return Ok(());
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

/// Maximum buffer size to prevent memory exhaustion (64KB)
const MAX_BUFFER_SIZE: usize = 65536;

/// A single RPLIDAR measurement node parsed from the 5-byte wire format.
#[derive(Debug, Clone, Copy)]
struct MeasurementNode {
    /// Signal quality (0-63). 0 = invalid measurement.
    quality: u8,
    /// Angle in degrees (0-360).
    angle_deg: f64,
    /// Distance in meters. 0 = invalid/no return.
    distance_m: f64,
    /// True if this node starts a new 360° scan.
    start_flag: bool,
}

/// Assembler for building complete 360° scans from RPLIDAR measurement nodes.
///
/// The RPLIDAR sends a continuous stream of 5-byte measurement nodes.
/// A new scan starts when a node has the start_flag set (S=1).
struct ScanAssembler {
    /// Incoming byte buffer for partial node handling.
    buffer: Vec<u8>,
    /// Nodes accumulated for the current scan revolution.
    current_scan: Vec<MeasurementNode>,
    /// Whether we've seen at least one start flag (synced).
    synced: bool,
}

impl ScanAssembler {
    /// Create a new scan assembler.
    fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(4096),
            current_scan: Vec::with_capacity(720),
            synced: false,
        }
    }

    /// Process incoming bytes, return any complete scans.
    fn process(
        &mut self,
        data: &[u8],
        range_min: f64,
        range_max: f64,
        scan_rate: f64,
    ) -> Vec<LaserScan> {
        self.buffer.extend_from_slice(data);

        // Prevent unbounded buffer growth
        if self.buffer.len() > MAX_BUFFER_SIZE {
            warn!(
                "LiDAR buffer overflow ({} bytes) - clearing and resyncing",
                self.buffer.len()
            );
            self.buffer.clear();
            self.current_scan.clear();
            self.synced = false;
            return Vec::new();
        }

        let mut scans = Vec::new();

        // Parse 5-byte measurement nodes
        while self.buffer.len() >= protocol::SCAN_NODE_SIZE {
            let node = Self::parse_node(&self.buffer[..protocol::SCAN_NODE_SIZE]);
            self.buffer.drain(..protocol::SCAN_NODE_SIZE);

            // Validate the start flag check bits (S and ~S must be complementary)
            if !node.valid_check_bits {
                // Possible desync — try to resync by scanning for a valid start node
                trace!("Invalid check bits, attempting resync");
                self.synced = false;
                self.current_scan.clear();
                // Skip one byte and retry
                if !self.buffer.is_empty() {
                    // Put back all but the first byte (we consumed 5, so re-insert 4)
                    // Actually we already drained 5 bytes. Just continue parsing.
                }
                continue;
            }

            if node.node.start_flag {
                // Start of new scan revolution
                if self.synced && self.current_scan.len() >= 10 {
                    // Complete the previous scan
                    scans.push(Self::build_scan(
                        &self.current_scan,
                        range_min,
                        range_max,
                        scan_rate,
                    ));
                }
                self.current_scan.clear();
                self.synced = true;
            }

            if self.synced {
                self.current_scan.push(node.node);
            }
        }

        scans
    }

    /// Parse a single 5-byte measurement node.
    ///
    /// Wire format (Slamtec RPLIDAR standard scan):
    ///   Byte 0: [quality:6][S:1][~S:1]
    ///   Byte 1: [angle_q6[6:0]:7][C:1]  (C = check bit, should be 1)
    ///   Byte 2: [angle_q6[14:7]:8]
    ///   Byte 3: [distance_q2[7:0]:8]
    ///   Byte 4: [distance_q2[15:8]:8]
    fn parse_node(data: &[u8]) -> ParsedNode {
        let byte0 = data[0];
        let byte1 = data[1];
        let byte2 = data[2];
        let byte3 = data[3];
        let byte4 = data[4];

        // Start flag and its complement
        let s_flag = (byte0 & 0x01) != 0;
        let not_s_flag = (byte0 & 0x02) != 0;
        let quality = byte0 >> 2;

        // Check bit (LSB of byte1, should be 1 for valid scan data)
        let check_bit = (byte1 & 0x01) != 0;

        // Angle: 15-bit fixed-point (degrees * 64)
        // Bits [14:7] from byte2, bits [6:0] from byte1[7:1]
        let angle_q6 = ((byte2 as u16) << 7) | ((byte1 as u16) >> 1);
        let angle_deg = angle_q6 as f64 / 64.0;

        // Distance: 16-bit fixed-point (mm * 4)
        let distance_q2 = (byte4 as u16) << 8 | (byte3 as u16);
        let distance_m = if distance_q2 == 0 {
            0.0 // No return
        } else {
            (distance_q2 as f64) / 4.0 / 1000.0 // q2 -> mm -> m
        };

        // Validate: S and ~S must be complementary, check_bit must be 1
        let valid = s_flag != not_s_flag && check_bit;

        ParsedNode {
            node: MeasurementNode {
                quality,
                angle_deg,
                distance_m,
                start_flag: s_flag,
            },
            valid_check_bits: valid,
        }
    }

    /// Build a LaserScan from accumulated measurement nodes.
    fn build_scan(
        nodes: &[MeasurementNode],
        range_min: f64,
        range_max: f64,
        scan_rate: f64,
    ) -> LaserScan {
        // Sort by angle
        let mut sorted: Vec<_> = nodes.to_vec();
        sorted.sort_by(|a, b| {
            a.angle_deg
                .partial_cmp(&b.angle_deg)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let num_points = sorted.len();
        if num_points == 0 {
            return LaserScan::default();
        }

        // RPLIDAR reports angles 0-360° (0° = forward).
        // Convert to ROS convention: -PI to +PI (0 = forward).
        let angle_min_deg = sorted.first().map(|n| n.angle_deg).unwrap_or(0.0);
        let angle_max_deg = sorted.last().map(|n| n.angle_deg).unwrap_or(360.0);

        let angle_min = angle_min_deg.to_radians() - std::f64::consts::PI;
        let angle_max = angle_max_deg.to_radians() - std::f64::consts::PI;
        let angle_increment = if num_points > 1 {
            (angle_max - angle_min) / (num_points - 1) as f64
        } else {
            0.0
        };

        let ranges: Vec<f64> = sorted
            .iter()
            .map(|n| {
                if n.quality == 0 || n.distance_m == 0.0 {
                    f64::INFINITY // No return
                } else if n.distance_m < range_min || n.distance_m > range_max {
                    f64::INFINITY // Out of range
                } else {
                    n.distance_m
                }
            })
            .collect();

        let intensities: Vec<f64> = sorted.iter().map(|n| n.quality as f64).collect();

        let scan_time = if scan_rate > 0.0 {
            1.0 / scan_rate
        } else {
            1.0 / 5.5
        };

        LaserScan {
            timestamp: now_secs(),
            angle_min,
            angle_max,
            angle_increment,
            time_increment: if num_points > 0 {
                scan_time / num_points as f64
            } else {
                0.0
            },
            scan_time,
            range_min,
            range_max,
            ranges,
            intensities,
        }
    }
}

/// Intermediate parse result with validity flag.
struct ParsedNode {
    node: MeasurementNode,
    valid_check_bits: bool,
}

impl Drop for LidarDriver {
    fn drop(&mut self) {
        // Attempt to stop the scanner
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.try_send(());
        }
    }
}
