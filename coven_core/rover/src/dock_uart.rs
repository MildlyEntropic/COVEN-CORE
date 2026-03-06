//! dock_uart.rs — COVEN Dock UART Communication
//!
//! Serial UART communication with dock via the COVEN 9-pin connector.
//! Implements the framed protocol from COVEN Interface Specification v0.3.
//!
//! Key design: NO WIRELESS COMMUNICATION while deployed.
//! The rover only communicates when physically docked via the 9-pin connector.
//!
//! Frame format (per Interface Spec v0.3 — COBS):
//!   [LEN_HI] [LEN_LO] [COBS-encoded data] [0x00]
//!   - LEN:   16-bit big-endian length of COBS-encoded data
//!   - DATA:  COBS encoding of [TYPE][PAYLOAD][CRC]
//!   - 0x00:  COBS frame terminator (guaranteed absent in encoded data)
//!   - CRC:   XOR of TYPE and all PAYLOAD bytes (pre-COBS)
//!
//! Message Types (from spec):
//!   0x01 — IDENTIFY_REQUEST (dock → rover)
//!   0x02 — IDENTIFY_REPLY   (rover → dock)
//!   0x03 — VERIFY_OK        (dock → rover)
//!   0x04 — VERIFY_FAIL      (dock → rover)
//!   0x10 — DATA_FRAME       (bidirectional)
//!   0x20 — MODULE_HEARTBEAT (rover → dock)
//!   0x30 — FAULT_ALERT      (rover → dock)
//!   0xFF — SYSTEM_PING      (bidirectional)
//!
//! Responsibilities:
//! - Detect physical dock connection via ID/Sense lines
//! - Open UART serial port when docked
//! - Frame and send messages per protocol spec
//! - Receive and parse framed messages with CRC validation
//! - Handle connection lifecycle (detect → identify → verify → operate)
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::time::{Duration, Instant};

// --- Third-party ---
use anyhow::{Context, Result};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tracing::{debug, error, info, trace, warn};

// --- Local ---
use crate::protocol::{DockMessage, RoverMessage};

// ------------------------
// --- Constants ---
// ------------------------

/// Maximum frame payload size (up from 255 in v0.2).
const MAX_PAYLOAD_SIZE: usize = 8192;
/// Maximum total frame size (len prefix + COBS overhead + terminator).
const MAX_FRAME_SIZE: usize = MAX_PAYLOAD_SIZE + 512;

/// Message types from Interface Specification v0.2.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    /// Dock requests identification (dock → rover).
    IdentifyRequest = 0x01,
    /// Rover responds with identity (rover → dock).
    IdentifyReply = 0x02,
    /// Dock confirms verification passed (dock → rover).
    VerifyOk = 0x03,
    /// Dock reports verification failed (dock → rover).
    VerifyFail = 0x04,
    /// Data frame for task/sensor data (bidirectional).
    DataFrame = 0x10,
    /// Periodic heartbeat from rover (rover → dock).
    ModuleHeartbeat = 0x20,
    /// Fault alert from rover (rover → dock).
    FaultAlert = 0x30,
    /// System ping for connection test (bidirectional).
    SystemPing = 0xFF,
}

impl MessageType {
    /// Convert from byte to MessageType.
    fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x01 => Some(Self::IdentifyRequest),
            0x02 => Some(Self::IdentifyReply),
            0x03 => Some(Self::VerifyOk),
            0x04 => Some(Self::VerifyFail),
            0x10 => Some(Self::DataFrame),
            0x20 => Some(Self::ModuleHeartbeat),
            0x30 => Some(Self::FaultAlert),
            0xFF => Some(Self::SystemPing),
            _ => None,
        }
    }
}

// ------------------------
// --- Connection State ---
// ------------------------

// ------------------------
// --- COBS Encode/Decode ---
// ------------------------

/// COBS encode: guarantees no 0x00 bytes in output.
fn cobs_encode(data: &[u8]) -> Vec<u8> {
    let mut output = Vec::with_capacity(data.len() + data.len() / 254 + 1);
    let mut idx = 0;

    while idx <= data.len() {
        // Find next zero byte (or end of data)
        let next_zero = data[idx..].iter().position(|&b| b == 0)
            .map(|p| idx + p)
            .unwrap_or(data.len());

        let mut block_len = next_zero - idx;

        // Handle blocks > 253 bytes
        while block_len > 253 {
            output.push(0xFF);
            output.extend_from_slice(&data[idx..idx + 254]);
            idx += 254;
            block_len -= 254;
        }

        // Write code byte + block
        output.push((block_len + 1) as u8);
        output.extend_from_slice(&data[idx..idx + block_len]);
        idx += block_len;

        // Skip past zero byte if present
        if idx < data.len() && data[idx] == 0 {
            idx += 1;
        } else {
            break;
        }
    }

    output
}

/// COBS decode: inverse of cobs_encode.
fn cobs_decode(data: &[u8]) -> Result<Vec<u8>> {
    let mut output = Vec::with_capacity(data.len());
    let mut idx = 0;

    while idx < data.len() {
        let code = data[idx] as usize;
        idx += 1;

        if code == 0 {
            anyhow::bail!("Unexpected zero in COBS data at offset {}", idx - 1);
        }

        let end = idx + code - 1;
        if end > data.len() {
            anyhow::bail!("COBS data truncated: need {} bytes from offset {}", code - 1, idx);
        }
        output.extend_from_slice(&data[idx..end]);
        idx = end;

        // If code < 0xFF and more data follows, insert implicit zero
        if code < 0xFF && idx < data.len() {
            output.push(0);
        }
    }

    Ok(output)
}

/// Physical docking state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // Protocol states — used as state machine transitions are wired in
pub enum DockState {
    /// Not physically connected to dock.
    Disconnected,
    /// Physical connection detected (ID line active).
    Detected,
    /// Identification handshake in progress.
    Identifying,
    /// Successfully verified with dock.
    Verified,
    /// Enabled for normal operations.
    Enabled,
    /// Connection rejected by dock.
    Rejected,
}

impl std::fmt::Display for DockState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Disconnected => write!(f, "DISCONNECTED"),
            Self::Detected => write!(f, "DETECTED"),
            Self::Identifying => write!(f, "IDENTIFYING"),
            Self::Verified => write!(f, "VERIFIED"),
            Self::Enabled => write!(f, "ENABLED"),
            Self::Rejected => write!(f, "REJECTED"),
        }
    }
}

// ------------------------
// --- Data Structures ---
// ------------------------

/// UART configuration for dock communication.
#[derive(Debug, Clone)]
pub struct DockUartConfig {
    /// Serial port device path (e.g., "/dev/ttyAMA0").
    pub port: String,
    /// Baud rate (default 115200).
    pub baud_rate: u32,
    /// Retry delay for connection attempts.
    pub retry_delay: Duration,
    /// Maximum retry attempts before giving up.
    pub max_retries: u32,
}

impl Default for DockUartConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyAMA0".to_string(),
            baud_rate: 115200,
            retry_delay: Duration::from_secs(1),
            max_retries: 10,
        }
    }
}

/// Dock UART connection manager.
pub struct DockUart {
    /// UART configuration.
    config: DockUartConfig,
    /// Current connection state.
    state: DockState,
    /// Sender for outgoing messages.
    outgoing_tx: Option<mpsc::Sender<RoverMessage>>,
    /// Receiver for incoming messages.
    incoming_rx: Option<mpsc::Receiver<DockMessage>>,
    /// Sender for stop signal.
    stop_tx: Option<mpsc::Sender<()>>,
    /// Time when we entered current state.
    state_entered_at: Instant,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl DockUart {
    /// Create a new dock UART connection manager.
    pub fn new(config: DockUartConfig) -> Self {
        Self {
            config,
            state: DockState::Disconnected,
            outgoing_tx: None,
            incoming_rx: None,
            stop_tx: None,
            state_entered_at: Instant::now(),
        }
    }

    /// Create with default configuration.
    #[allow(dead_code)]
    pub fn with_defaults() -> Self {
        Self::new(DockUartConfig::default())
    }

    /// Start the connection manager and spawn UART task.
    ///
    /// Note: This only opens the UART port. Actual communication only
    /// happens when physically docked (dock provides +5V standby power).
    pub async fn connect(&mut self) -> Result<()> {
        if self.state != DockState::Disconnected {
            return Ok(());
        }

        let config = self.config.clone();

        // Create channels
        let (outgoing_tx, outgoing_rx) = mpsc::channel::<RoverMessage>(100);
        let (incoming_tx, incoming_rx) = mpsc::channel::<DockMessage>(100);
        let (stop_tx, stop_rx) = mpsc::channel::<()>(1);

        self.outgoing_tx = Some(outgoing_tx);
        self.incoming_rx = Some(incoming_rx);
        self.stop_tx = Some(stop_tx);

        // Spawn UART task
        tokio::spawn(async move {
            uart_loop(config, outgoing_rx, incoming_tx, stop_rx).await;
        });

        self.transition_to(DockState::Detected);
        Ok(())
    }

    /// Send a message to the dock.
    pub async fn send(&self, message: RoverMessage) -> Result<()> {
        if let Some(tx) = &self.outgoing_tx {
            tx.send(message)
                .await
                .context("Failed to queue message for sending")?;
            Ok(())
        } else {
            anyhow::bail!("Not connected to dock UART")
        }
    }

    /// Try to receive a message from the dock (non-blocking).
    pub fn try_recv(&mut self) -> Option<DockMessage> {
        if let Some(rx) = &mut self.incoming_rx {
            rx.try_recv().ok()
        } else {
            None
        }
    }

    /// Receive a message with timeout.
    #[allow(dead_code)]
    pub async fn recv_timeout(&mut self, timeout: Duration) -> Option<DockMessage> {
        if let Some(rx) = &mut self.incoming_rx {
            tokio::time::timeout(timeout, rx.recv())
                .await
                .ok()
                .flatten()
        } else {
            None
        }
    }

    /// Get current dock connection state.
    #[allow(dead_code)]
    pub fn state(&self) -> DockState {
        self.state
    }

    /// Check if physically connected to dock.
    #[allow(dead_code)]
    pub fn is_docked(&self) -> bool {
        matches!(
            self.state,
            DockState::Detected
                | DockState::Identifying
                | DockState::Verified
                | DockState::Enabled
        )
    }

    /// Disconnect from dock.
    #[allow(dead_code)]
    pub async fn disconnect(&mut self) {
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.send(()).await;
        }
        self.transition_to(DockState::Disconnected);
    }

    /// Transition to a new state.
    fn transition_to(&mut self, new_state: DockState) {
        if self.state != new_state {
            info!(from = %self.state, to = %new_state, "Dock state transition");
            self.state = new_state;
            self.state_entered_at = Instant::now();
        }
    }
}

impl Drop for DockUart {
    fn drop(&mut self) {
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.try_send(());
        }
    }
}

// ------------------------
// --- UART Loop ---
// ------------------------

/// Main UART communication loop.
async fn uart_loop(
    config: DockUartConfig,
    mut outgoing_rx: mpsc::Receiver<RoverMessage>,
    incoming_tx: mpsc::Sender<DockMessage>,
    mut stop_rx: mpsc::Receiver<()>,
) {
    let mut retry_count = 0;

    info!("--- Dock UART Communication ---");
    info!("  Port: {}", config.port);
    info!("  Baud rate: {}", config.baud_rate);
    info!("  NOTE: Communication only active when physically docked");

    loop {
        // Check for stop signal
        if stop_rx.try_recv().is_ok() {
            info!("UART loop stopping (received stop signal)");
            break;
        }

        // Try to open serial port
        info!(
            "Opening UART port {}... (attempt #{})",
            config.port,
            retry_count + 1
        );

        let port_result = tokio_serial::new(&config.port, config.baud_rate)
            .timeout(Duration::from_millis(100))
            .open_native_async();

        match port_result {
            Ok(port) => {
                retry_count = 0;
                info!("[OK] UART port opened: {}", config.port);

                // Handle connection
                let result =
                    handle_uart_connection(port, &mut outgoing_rx, &incoming_tx, &mut stop_rx)
                        .await;

                match result {
                    Ok(()) => {
                        info!("UART connection closed gracefully");
                    }
                    Err(e) => {
                        warn!("UART connection error: {}", e);
                    }
                }
            }
            Err(e) => {
                retry_count += 1;
                error!("!!! UART PORT OPEN FAILED !!!");
                error!("  Error: {}", e);
                error!("  Port: {}", config.port);
                error!("  Attempt: #{}", retry_count);
                error!("  Troubleshooting:");
                error!("    1. Check serial port exists: ls -la /dev/ttyAMA*");
                error!("    2. Check user is in dialout group: groups $USER");
                error!("    3. Check port permissions: ls -la {}", config.port);
                error!("    4. Ensure no other process using port: lsof {}", config.port);

                if retry_count >= config.max_retries {
                    error!("Max retries exceeded - giving up on UART connection");
                    break;
                }
            }
        }

        // Wait before retry
        tokio::select! {
            _ = stop_rx.recv() => {
                info!("UART loop stopping during backoff");
                break;
            }
            _ = tokio::time::sleep(config.retry_delay) => {}
        }
    }

    info!("Dock UART communication shutdown complete");
}

/// Handle an established UART connection.
async fn handle_uart_connection(
    mut port: SerialStream,
    outgoing_rx: &mut mpsc::Receiver<RoverMessage>,
    incoming_tx: &mpsc::Sender<DockMessage>,
    stop_rx: &mut mpsc::Receiver<()>,
) -> Result<()> {
    let mut read_buf = [0u8; 4096];
    let mut frame_buf = Vec::with_capacity(MAX_FRAME_SIZE);

    // COBS frame parser state machine
    #[derive(Debug, Clone, Copy, PartialEq)]
    enum ParseState {
        WaitLenHi,
        WaitLenLo,
        Reading,
        WaitTerm,
    }
    let mut parse_state = ParseState::WaitLenHi;
    let mut len_hi: u8 = 0;
    let mut expected_len: usize = 0;

    // Statistics
    let mut frames_sent: u64 = 0;
    let mut frames_received: u64 = 0;
    let mut crc_errors: u64 = 0;

    trace!("UART handler started, entering message loop (COBS framing)");

    loop {
        tokio::select! {
            // Check for stop signal
            _ = stop_rx.recv() => {
                info!("UART handler stopping (stop signal received)");
                info!("  Stats: sent={} frames, recv={} frames, crc_errors={}",
                    frames_sent, frames_received, crc_errors);
                return Ok(());
            }

            // Read from UART
            result = port.read(&mut read_buf) => {
                match result {
                    Ok(0) => {
                        continue;
                    }
                    Ok(n) => {
                        // Process received bytes through COBS state machine
                        for &byte in &read_buf[..n] {
                            match parse_state {
                                ParseState::WaitLenHi => {
                                    len_hi = byte;
                                    parse_state = ParseState::WaitLenLo;
                                }
                                ParseState::WaitLenLo => {
                                    expected_len = ((len_hi as usize) << 8) | (byte as usize);
                                    if expected_len == 0 || expected_len > MAX_FRAME_SIZE {
                                        parse_state = ParseState::WaitLenHi;
                                        continue;
                                    }
                                    frame_buf.clear();
                                    parse_state = ParseState::Reading;
                                }
                                ParseState::Reading => {
                                    frame_buf.push(byte);
                                    if frame_buf.len() >= expected_len {
                                        parse_state = ParseState::WaitTerm;
                                    }
                                }
                                ParseState::WaitTerm => {
                                    if byte == 0x00 {
                                        // Complete frame — parse it
                                        match parse_frame(&frame_buf) {
                                            Ok(Some(msg)) => {
                                                frames_received += 1;
                                                debug!(
                                                    frame = frames_received,
                                                    "Received dock message"
                                                );
                                                if incoming_tx.send(msg).await.is_err() {
                                                    warn!("Message receiver dropped");
                                                    return Ok(());
                                                }
                                            }
                                            Ok(None) => {
                                                trace!("Unrecognized message type in frame");
                                            }
                                            Err(e) => {
                                                crc_errors += 1;
                                                warn!("Frame parse error: {} (total CRC errors: {})", e, crc_errors);
                                            }
                                        }
                                    } else {
                                        crc_errors += 1;
                                        warn!("Missing COBS terminator");
                                    }
                                    frame_buf.clear();
                                    parse_state = ParseState::WaitLenHi;
                                }
                            }
                        }
                    }
                    Err(e) => {
                        // Check if it's just a timeout (expected)
                        if e.kind() == std::io::ErrorKind::TimedOut {
                            continue;
                        }
                        error!("UART read error: {}", e);
                        return Err(e.into());
                    }
                }
            }

            // Write to UART
            Some(msg) = outgoing_rx.recv() => {
                match build_frame(&msg) {
                    Ok(frame) => {
                        trace!("Sending frame ({} bytes)", frame.len());
                        match port.write_all(&frame).await {
                            Ok(()) => {
                                if let Err(e) = port.flush().await {
                                    error!("UART flush error: {}", e);
                                    return Err(e.into());
                                }
                                frames_sent += 1;
                                debug!(frame = frames_sent, "Sent rover message");
                            }
                            Err(e) => {
                                error!("UART write error: {}", e);
                                return Err(e.into());
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Failed to build frame: {}", e);
                    }
                }
            }
        }
    }
}

// ------------------------
// --- Frame Building ---
// ------------------------

/// Build a COBS-framed UART message from a RoverMessage.
///
/// Wire format: [LEN_HI][LEN_LO][COBS data][0x00]
fn build_frame(msg: &RoverMessage) -> Result<Vec<u8>> {
    // Determine message type and payload
    let (msg_type, payload) = encode_message(msg)?;

    if payload.len() > MAX_PAYLOAD_SIZE {
        anyhow::bail!(
            "Payload too large: {} bytes (max {})",
            payload.len(),
            MAX_PAYLOAD_SIZE
        );
    }

    // Build raw block: [TYPE] [PAYLOAD] [CRC]
    let mut crc: u8 = msg_type;
    for &b in &payload {
        crc ^= b;
    }

    let mut raw = Vec::with_capacity(payload.len() + 2);
    raw.push(msg_type);
    raw.extend_from_slice(&payload);
    raw.push(crc);

    // COBS encode (no zeros in output)
    let encoded = cobs_encode(&raw);

    // Build frame: [LEN_HI][LEN_LO][COBS data][0x00]
    let len = encoded.len() as u16;
    let mut frame = Vec::with_capacity(encoded.len() + 3);
    frame.push((len >> 8) as u8);
    frame.push(len as u8);
    frame.extend_from_slice(&encoded);
    frame.push(0x00); // COBS terminator

    Ok(frame)
}

/// Encode a RoverMessage into message type and payload bytes.
fn encode_message(msg: &RoverMessage) -> Result<(u8, Vec<u8>)> {
    match msg {
        RoverMessage::IdentifyRep {
            module_id,
            module_type,
            firmware,
            battery_level,
            status,
            capabilities,
        } => {
            // IDENTIFY_REPLY payload format (from spec):
            // "COV" magic + module_type(1) + revision(1) + extended data
            // We extend this with: module_id, firmware, battery, status, capabilities
            let mut payload = Vec::new();

            // Magic bytes "COV"
            payload.extend_from_slice(b"COV");

            // Module type byte (simplified)
            let type_byte = match module_type.as_str() {
                "ReconRover" => 0x01,
                "CargoRover" => 0x02,
                "DrillRover" => 0x03,
                "ArduinoBot" => 0x04,
                _ => 0x00,
            };
            payload.push(type_byte);

            // Revision byte (firmware major version)
            let rev = firmware
                .split('.')
                .next()
                .and_then(|s| s.parse::<u8>().ok())
                .unwrap_or(0);
            payload.push(rev);

            // Extended: module_id length + string
            let id_bytes = module_id.as_bytes();
            payload.push(id_bytes.len() as u8);
            payload.extend_from_slice(id_bytes);

            // Battery level (0-100 as single byte)
            payload.push(battery_level.clamp(0.0, 100.0) as u8);

            // Status length + string
            let status_bytes = status.as_bytes();
            payload.push(status_bytes.len().min(255) as u8);
            payload.extend_from_slice(&status_bytes[..status_bytes.len().min(255)]);

            // Capabilities bitmask (1 byte)
            payload.push(*capabilities);

            Ok((MessageType::IdentifyReply as u8, payload))
        }

        RoverMessage::VerifyRep {
            module_id,
            success,
            failed_checks,
            note,
        } => {
            // Use DATA_FRAME for verification response
            let mut payload = Vec::new();

            // Subtype: VERIFY_REP
            payload.push(0x01);

            // Module ID
            let id_bytes = module_id.as_bytes();
            payload.push(id_bytes.len() as u8);
            payload.extend_from_slice(id_bytes);

            // Success flag
            payload.push(if *success { 1 } else { 0 });

            // Failed checks count + strings
            payload.push(failed_checks.len().min(255) as u8);
            for check in failed_checks.iter().take(10) {
                let check_bytes = check.as_bytes();
                payload.push(check_bytes.len().min(32) as u8);
                payload.extend_from_slice(&check_bytes[..check_bytes.len().min(32)]);
            }

            // Note (truncated to fit)
            let note_bytes = note.as_bytes();
            payload.push(note_bytes.len().min(100) as u8);
            payload.extend_from_slice(&note_bytes[..note_bytes.len().min(100)]);

            Ok((MessageType::DataFrame as u8, payload))
        }

        RoverMessage::Heartbeat {
            module_id,
            battery_pct,
            mission_status,
            x,
            y,
            theta,
        } => {
            let mut payload = Vec::new();

            // Module ID
            let id_bytes = module_id.as_bytes();
            payload.push(id_bytes.len() as u8);
            payload.extend_from_slice(id_bytes);

            // Battery (0-100)
            payload.push(battery_pct.clamp(0.0, 100.0) as u8);

            // Mission status byte
            let status_byte = match mission_status.as_str() {
                "IDLE" => 0x00,
                "ACTIVE" => 0x01,
                "STARTUP" => 0x02,
                "RETURNING" => 0x03,
                _ => 0xFF,
            };
            payload.push(status_byte);

            // Position as fixed-point (mm precision)
            let x_mm = (x * 1000.0) as i32;
            let y_mm = (y * 1000.0) as i32;
            let theta_mrad = (theta * 1000.0) as i16;

            payload.extend_from_slice(&x_mm.to_le_bytes());
            payload.extend_from_slice(&y_mm.to_le_bytes());
            payload.extend_from_slice(&theta_mrad.to_le_bytes());

            Ok((MessageType::ModuleHeartbeat as u8, payload))
        }

        RoverMessage::TaskAck { .. }
        | RoverMessage::TaskStart { .. }
        | RoverMessage::TaskComplete { .. } => {
            // These use DATA_FRAME with JSON payload for flexibility
            let json = serde_json::to_vec(msg)?;
            let mut payload = Vec::new();
            payload.push(0x10); // Subtype: TASK_MESSAGE
            payload.extend_from_slice(&json);
            Ok((MessageType::DataFrame as u8, payload))
        }

        RoverMessage::DataBatch { .. }
        | RoverMessage::ScanData { .. }
        | RoverMessage::OdomData { .. } => {
            // Large data uses chunked DATA_FRAME with JSON
            // Note: For very large batches, this should be chunked
            let json = serde_json::to_vec(msg)?;
            let mut payload = Vec::new();
            payload.push(0x20); // Subtype: SENSOR_DATA
            payload.extend_from_slice(&json);
            Ok((MessageType::DataFrame as u8, payload))
        }
    }
}

// ------------------------
// --- Frame Parsing ---
// ------------------------

/// Parse a COBS-encoded frame into a DockMessage.
///
/// Input is the COBS-encoded data (without length prefix or terminator).
fn parse_frame(cobs_data: &[u8]) -> Result<Option<DockMessage>> {
    // COBS decode
    let raw = cobs_decode(cobs_data)?;

    // Minimum: TYPE + CRC = 2 bytes
    if raw.len() < 2 {
        anyhow::bail!("Decoded frame too short: {} bytes", raw.len());
    }

    let msg_type = raw[0];
    let payload = &raw[1..raw.len() - 1];
    let received_crc = raw[raw.len() - 1];

    // Calculate expected CRC (XOR of type and payload)
    let mut expected_crc: u8 = msg_type;
    for &b in payload {
        expected_crc ^= b;
    }

    if received_crc != expected_crc {
        anyhow::bail!(
            "CRC mismatch: expected 0x{:02X}, got 0x{:02X}",
            expected_crc,
            received_crc
        );
    }

    // Parse based on message type
    let msg_type_enum = MessageType::from_byte(msg_type);

    match msg_type_enum {
        Some(MessageType::IdentifyRequest) => parse_identify_request(payload),
        Some(MessageType::VerifyOk) => parse_verify_ok(payload),
        Some(MessageType::VerifyFail) => parse_verify_fail(payload),
        Some(MessageType::DataFrame) => parse_data_frame(payload),
        Some(MessageType::SystemPing) => {
            trace!("Received SYSTEM_PING");
            Ok(None)
        }
        _ => {
            trace!("Unknown message type: 0x{:02X}", msg_type);
            Ok(None)
        }
    }
}

/// Parse IDENTIFY_REQUEST payload.
fn parse_identify_request(payload: &[u8]) -> Result<Option<DockMessage>> {
    // Expected: dock_id_len(1) + dock_id + coven_name_len(1) + coven_name + assigned_name_len(1) + assigned_name
    if payload.is_empty() {
        anyhow::bail!("Empty IDENTIFY_REQUEST payload");
    }

    let mut pos = 0;

    // Dock ID
    let dock_id_len = payload[pos] as usize;
    pos += 1;
    if pos + dock_id_len > payload.len() {
        anyhow::bail!("IDENTIFY_REQUEST: dock_id length exceeds payload");
    }
    let dock_id = String::from_utf8_lossy(&payload[pos..pos + dock_id_len]).to_string();
    pos += dock_id_len;

    // Coven name
    if pos >= payload.len() {
        anyhow::bail!("IDENTIFY_REQUEST: missing coven_name");
    }
    let coven_name_len = payload[pos] as usize;
    pos += 1;
    if pos + coven_name_len > payload.len() {
        anyhow::bail!("IDENTIFY_REQUEST: coven_name length exceeds payload");
    }
    let coven_name = String::from_utf8_lossy(&payload[pos..pos + coven_name_len]).to_string();
    pos += coven_name_len;

    // Assigned name (optional)
    let assigned_name = if pos < payload.len() {
        let assigned_len = payload[pos] as usize;
        pos += 1;
        if pos + assigned_len <= payload.len() {
            String::from_utf8_lossy(&payload[pos..pos + assigned_len]).to_string()
        } else {
            String::new()
        }
    } else {
        String::new()
    };

    Ok(Some(DockMessage::IdentifyReq {
        dock_id,
        dock_name: coven_name,
        assigned_name,
    }))
}

/// Parse VERIFY_OK payload.
fn parse_verify_ok(payload: &[u8]) -> Result<Option<DockMessage>> {
    // dock_id_len(1) + dock_id + module_id_len(1) + module_id
    if payload.is_empty() {
        anyhow::bail!("Empty VERIFY_OK payload");
    }

    let mut pos = 0;

    // Dock ID
    let dock_id_len = payload[pos] as usize;
    pos += 1;
    if pos + dock_id_len > payload.len() {
        anyhow::bail!("VERIFY_OK: dock_id length exceeds payload");
    }
    let dock_id = String::from_utf8_lossy(&payload[pos..pos + dock_id_len]).to_string();
    pos += dock_id_len;

    // Module ID
    if pos >= payload.len() {
        anyhow::bail!("VERIFY_OK: missing module_id");
    }
    let module_id_len = payload[pos] as usize;
    pos += 1;
    let module_id = if pos + module_id_len <= payload.len() {
        String::from_utf8_lossy(&payload[pos..pos + module_id_len]).to_string()
    } else {
        String::new()
    };

    Ok(Some(DockMessage::VerifyReq { dock_id, module_id, accepted: true }))
}

/// Parse VERIFY_FAIL payload.
fn parse_verify_fail(payload: &[u8]) -> Result<Option<DockMessage>> {
    // Same payload format as VERIFY_OK, but dock is rejecting this module
    if payload.is_empty() {
        anyhow::bail!("Empty VERIFY_FAIL payload");
    }

    let mut pos = 0;

    // Dock ID
    let dock_id_len = payload[pos] as usize;
    pos += 1;
    if pos + dock_id_len > payload.len() {
        anyhow::bail!("VERIFY_FAIL: dock_id length exceeds payload");
    }
    let dock_id = String::from_utf8_lossy(&payload[pos..pos + dock_id_len]).to_string();
    pos += dock_id_len;

    // Module ID
    if pos >= payload.len() {
        anyhow::bail!("VERIFY_FAIL: missing module_id");
    }
    let module_id_len = payload[pos] as usize;
    pos += 1;
    let module_id = if pos + module_id_len <= payload.len() {
        String::from_utf8_lossy(&payload[pos..pos + module_id_len]).to_string()
    } else {
        String::new()
    };

    Ok(Some(DockMessage::VerifyReq { dock_id, module_id, accepted: false }))
}

/// Parse DATA_FRAME payload (task requests, etc.).
fn parse_data_frame(payload: &[u8]) -> Result<Option<DockMessage>> {
    if payload.is_empty() {
        anyhow::bail!("Empty DATA_FRAME payload");
    }

    let subtype = payload[0];
    let data = &payload[1..];

    match subtype {
        0x10 => {
            // TASK_MESSAGE - JSON encoded
            if let Ok(msg) = serde_json::from_slice::<DockMessage>(data) {
                Ok(Some(msg))
            } else {
                trace!("Failed to parse TASK_MESSAGE JSON");
                Ok(None)
            }
        }
        0x30 => {
            // CMD_VEL - binary encoded
            if data.len() >= 8 {
                let linear = f32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f64;
                let angular = f32::from_le_bytes([data[4], data[5], data[6], data[7]]) as f64;
                Ok(Some(DockMessage::CmdVel { linear, angular }))
            } else {
                anyhow::bail!("CMD_VEL data too short");
            }
        }
        0x40 => {
            // ENABLE_POWER - binary encoded
            if data.len() >= 5 {
                let voltage = data[0] as u32;
                let duration =
                    f32::from_le_bytes([data[1], data[2], data[3], data[4]]) as f64;
                Ok(Some(DockMessage::EnablePower { voltage, duration }))
            } else {
                anyhow::bail!("ENABLE_POWER data too short");
            }
        }
        _ => {
            trace!("Unknown DATA_FRAME subtype: 0x{:02X}", subtype);
            Ok(None)
        }
    }
}

