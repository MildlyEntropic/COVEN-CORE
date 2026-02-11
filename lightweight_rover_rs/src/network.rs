//! network.rs — COVEN Dock Communication
//!
//! TCP client for dock communication with automatic reconnection.
//!
//! Responsibilities:
//! - Establish and maintain TCP connection to dock
//! - Handle automatic reconnection with exponential backoff
//! - Serialize and send rover messages
//! - Receive and parse dock messages
//! - Provide async message passing via channels
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
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tracing::{debug, error, info, trace, warn};

// --- Local ---
use crate::protocol::{DockMessage, RoverMessage};
use crate::utils::IoStats;

// ------------------------
// --- Data Structures ---
// ------------------------

/// Connection state for dock communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// Not connected to dock.
    Disconnected,
    /// Connection attempt in progress.
    Connecting,
    /// Successfully connected to dock.
    Connected,
}

/// Dock connection manager with async message passing.
pub struct DockConnection {
    /// Dock IP address.
    address: String,
    /// Dock TCP port.
    port: u16,
    /// Current connection state.
    state: ConnectionState,
    /// Sender for outgoing messages to dock.
    outgoing_tx: Option<mpsc::Sender<RoverMessage>>,
    /// Receiver for incoming messages from dock.
    incoming_rx: Option<mpsc::Receiver<DockMessage>>,
    /// Sender for stop signal to connection task.
    stop_tx: Option<mpsc::Sender<()>>,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl DockConnection {
    /// Create a new dock connection (not yet connected).
    pub fn new(address: &str, port: u16) -> Self {
        Self {
            address: address.to_string(),
            port,
            state: ConnectionState::Disconnected,
            outgoing_tx: None,
            incoming_rx: None,
            stop_tx: None,
        }
    }

    /// Start the connection manager and spawn connection task.
    pub async fn connect(&mut self) -> Result<()> {
        if self.state == ConnectionState::Connected {
            return Ok(());
        }

        let address = self.address.clone();
        let port = self.port;

        // Create channels
        let (outgoing_tx, outgoing_rx) = mpsc::channel::<RoverMessage>(100);
        let (incoming_tx, incoming_rx) = mpsc::channel::<DockMessage>(100);
        let (stop_tx, stop_rx) = mpsc::channel::<()>(1);

        self.outgoing_tx = Some(outgoing_tx);
        self.incoming_rx = Some(incoming_rx);
        self.stop_tx = Some(stop_tx);

        // Spawn connection task
        tokio::spawn(async move {
            connection_loop(address, port, outgoing_rx, incoming_tx, stop_rx).await;
        });

        self.state = ConnectionState::Connecting;
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
            anyhow::bail!("Not connected")
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

    /// Receive a message from the dock (blocking with timeout).
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

    /// Get current connection state.
    #[allow(dead_code)]
    pub fn state(&self) -> ConnectionState {
        self.state
    }

    /// Check if connected to dock.
    #[allow(dead_code)]
    pub fn is_connected(&self) -> bool {
        self.state == ConnectionState::Connected
    }

    /// Disconnect from dock.
    #[allow(dead_code)]
    pub async fn disconnect(&mut self) {
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.send(()).await;
        }
        self.state = ConnectionState::Disconnected;
    }
}

// ------------------------
// --- Connection Loop ---
// ------------------------

/// Main connection loop with auto-reconnect.
async fn connection_loop(
    address: String,
    port: u16,
    mut outgoing_rx: mpsc::Receiver<RoverMessage>,
    incoming_tx: mpsc::Sender<DockMessage>,
    mut stop_rx: mpsc::Receiver<()>,
) {
    let mut retry_delay = Duration::from_secs(1);
    let max_retry_delay = Duration::from_secs(30);
    let mut connection_attempts: u64 = 0;
    let mut total_connections: u64 = 0;
    let mut last_connection_time: Option<std::time::Instant> = None;

    info!("--- Network Connection Manager ---");
    info!("  Target dock: {}:{}", address, port);
    info!(
        "  Retry strategy: exponential backoff (1s to {}s)",
        max_retry_delay.as_secs()
    );

    loop {
        // Check for stop signal
        if stop_rx.try_recv().is_ok() {
            info!("Connection loop stopping (received stop signal)");
            break;
        }

        connection_attempts += 1;

        // Try to connect
        info!(
            "Connecting to dock {}:{}... (attempt #{})",
            address, port, connection_attempts
        );

        let connect_start = std::time::Instant::now();
        let connect_result = tokio::time::timeout(
            Duration::from_secs(10),
            TcpStream::connect(format!("{}:{}", address, port)),
        )
        .await;

        match connect_result {
            Ok(Ok(stream)) => {
                total_connections += 1;
                let connect_elapsed = connect_start.elapsed();

                // Log connection details
                let local_addr = stream.local_addr().ok();
                let peer_addr = stream.peer_addr().ok();
                info!("[OK] Connected to dock in {:?}", connect_elapsed);
                if let Some(local) = local_addr {
                    debug!("  Local endpoint: {}", local);
                }
                if let Some(peer) = peer_addr {
                    debug!("  Remote endpoint: {}", peer);
                }
                info!("  Total successful connections: {}", total_connections);

                // Calculate uptime if we had a previous connection
                if let Some(last_time) = last_connection_time {
                    let downtime = last_time.elapsed();
                    if downtime > Duration::from_secs(5) {
                        info!(
                            "  Reconnected after {:.1}s downtime",
                            downtime.as_secs_f64()
                        );
                    }
                }

                retry_delay = Duration::from_secs(1); // Reset retry delay
                connection_attempts = 0; // Reset attempts on success

                // Handle connection
                let connection_start = std::time::Instant::now();
                let result =
                    handle_connection(stream, &mut outgoing_rx, &incoming_tx, &mut stop_rx).await;

                let session_duration = connection_start.elapsed();

                match result {
                    Ok(()) => {
                        info!(
                            "Connection closed gracefully (session: {:.1}s)",
                            session_duration.as_secs_f64()
                        );
                    }
                    Err(e) => {
                        warn!("!!! CONNECTION ERROR !!!");
                        warn!("  Error: {}", e);
                        warn!("  Session duration: {:.1}s", session_duration.as_secs_f64());
                        warn!("  Will attempt to reconnect...");
                    }
                }

                last_connection_time = Some(std::time::Instant::now());
            }
            Ok(Err(e)) => {
                error!("!!! DOCK CONNECTION FAILED !!!");
                error!("  Error: {}", e);
                error!("  Target: {}:{}", address, port);
                error!("  Attempt: #{}", connection_attempts);
                error!("  Troubleshooting:");
                error!("    1. Check dock is running: systemctl status coven-dock");
                error!("    2. Check network: ping {}", address);
                error!("    3. Check port is open: nc -zv {} {}", address, port);
                error!("    4. Check firewall: sudo ufw status");
                error!("  Retrying in {:?}...", retry_delay);
            }
            Err(_) => {
                error!("!!! CONNECTION TIMEOUT !!!");
                error!("  Timeout after 10 seconds trying to connect");
                error!("  Target: {}:{}", address, port);
                error!("  Attempt: #{}", connection_attempts);
                error!("  This usually means:");
                error!("    - Dock is not reachable (network issue)");
                error!("    - Firewall blocking connection");
                error!("    - Dock service not running");
                error!("  Retrying in {:?}...", retry_delay);
            }
        }

        // Wait before reconnecting
        tokio::select! {
            _ = stop_rx.recv() => {
                info!("Connection loop stopping (received stop signal during backoff)");
                break;
            }
            _ = tokio::time::sleep(retry_delay) => {
                trace!("Backoff complete, attempting reconnection...");
            }
        }

        // Exponential backoff
        retry_delay = (retry_delay * 2).min(max_retry_delay);

        // Periodic status update during extended reconnection attempts
        if connection_attempts > 0 && connection_attempts.is_multiple_of(10) {
            warn!(
                "Still trying to connect to dock (attempt #{}, backoff: {:?})",
                connection_attempts, retry_delay
            );
        }
    }

    info!("Network connection manager shutdown complete");
    info!("  Total connection attempts: {}", connection_attempts);
    info!("  Total successful connections: {}", total_connections);
}

/// Handle an established connection.
async fn handle_connection(
    stream: TcpStream,
    outgoing_rx: &mut mpsc::Receiver<RoverMessage>,
    incoming_tx: &mpsc::Sender<DockMessage>,
    stop_rx: &mut mpsc::Receiver<()>,
) -> Result<()> {
    let (reader, mut writer) = stream.into_split();
    let mut reader = BufReader::new(reader);
    let mut line_buf = String::new();

    // Message statistics
    let mut stats = IoStats::new();

    trace!("Connection handler started, entering message loop");

    loop {
        // Periodic stats logging (every 60 seconds)
        if stats.should_log(60.0) {
            debug!(
                "Connection stats: sent={} msgs ({} bytes), recv={} msgs ({} bytes), parse_errors={}",
                stats.messages_sent, stats.bytes_sent, stats.messages_received, stats.bytes_received, stats.parse_errors
            );
        }

        tokio::select! {
            // Check for stop signal
            _ = stop_rx.recv() => {
                info!("Connection handler stopping (stop signal received)");
                info!(
                    "  Final stats: sent={} msgs, recv={} msgs, parse_errors={}",
                    stats.messages_sent, stats.messages_received, stats.parse_errors
                );
                return Ok(());
            }

            // Read from socket
            result = reader.read_line(&mut line_buf) => {
                match result {
                    Ok(0) => {
                        // Connection closed by remote
                        info!("Connection closed by dock (EOF)");
                        info!(
                            "  Session stats: sent={} msgs, recv={} msgs, parse_errors={}",
                            stats.messages_sent, stats.messages_received, stats.parse_errors
                        );
                        return Ok(());
                    }
                    Ok(n) => {
                        let msg = line_buf.trim();
                        if !msg.is_empty() {
                            trace!("Raw recv ({} bytes): {}", n, msg);
                            if let Some(parsed) = DockMessage::parse(msg) {
                                stats.record_receive(n);
                                debug!(
                                    msg_type = ?std::mem::discriminant(&parsed),
                                    "Received message #{}",
                                    stats.messages_received
                                );
                                if incoming_tx.send(parsed).await.is_err() {
                                    // Receiver dropped
                                    warn!("Message receiver dropped, closing connection");
                                    return Ok(());
                                }
                            } else {
                                stats.record_error();
                                warn!("Failed to parse message from dock: {}", msg);
                                if stats.parse_errors == 1 {
                                    warn!("  (First parse error - may indicate protocol mismatch)");
                                }
                                if stats.parse_errors.is_multiple_of(10) {
                                    warn!("  Total parse errors: {} (check protocol compatibility)", stats.parse_errors);
                                }
                            }
                        }
                        line_buf.clear();
                    }
                    Err(e) => {
                        error!("Socket read error: {}", e);
                        error!(
                            "  Session stats before error: sent={}, recv={}, errors={}",
                            stats.messages_sent, stats.messages_received, stats.parse_errors
                        );
                        return Err(e.into());
                    }
                }
            }

            // Write to socket
            Some(msg) = outgoing_rx.recv() => {
                let wire = msg.to_wire();
                let wire_len = wire.len();
                trace!("Raw send ({} bytes): {}", wire_len, wire);

                match writer.write_all(wire.as_bytes()).await {
                    Ok(()) => {
                        if let Err(e) = writer.write_all(b"\n").await {
                            error!("Failed to write newline: {}", e);
                            return Err(e.into());
                        }
                        if let Err(e) = writer.flush().await {
                            error!("Failed to flush socket: {}", e);
                            return Err(e.into());
                        }
                        stats.record_send(wire_len + 1);
                        debug!(
                            msg_type = ?std::mem::discriminant(&msg),
                            "Sent message #{}",
                            stats.messages_sent
                        );
                    }
                    Err(e) => {
                        error!("Socket write error: {}", e);
                        error!("  Failed message: {}", wire);
                        return Err(e.into());
                    }
                }
            }
        }
    }
}

impl Drop for DockConnection {
    fn drop(&mut self) {
        if let Some(tx) = self.stop_tx.take() {
            let _ = tx.try_send(());
        }
    }
}
