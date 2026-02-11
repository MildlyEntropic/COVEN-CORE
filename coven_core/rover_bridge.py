"""
rover_bridge.py — COVEN Lightweight Rover Bridge

TCP server that bridges lightweight Rust rovers to the ROS2 ecosystem.
Handles the COVEN handshake protocol and translates between:
- Rover sensor data (TCP) → ROS2 topics (LaserScan, Odometry)
- ROS2 cmd_vel → TCP velocity commands to rover

This node runs on the dock and manages connections from multiple rovers.

Protocol:
- Simple messages use colon-delimited format: "MSG_TYPE:field1:field2:..."
- Complex messages (scan/odom data) use JSON

Author: Alexander Shultis
Date: January 2025
"""

import asyncio
import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from coven_core.common import (
    get_coven_name, get_witch_name,
    mark_witch_missing, mark_witch_active, is_witch_known, get_witch_status
)
from coven_core.task_auctioneer import (
    TaskAuctioneer, Mission, TaskType, PayloadType,
    RoverStatus as AuctionRoverStatus, generate_exploration_grid
)


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
DEFAULT_PORT = 5555
HEARTBEAT_TIMEOUT = 5.0  # seconds before marking rover as disconnected
CMD_VEL_TIMEOUT = 0.5    # seconds - rover stops if no cmd_vel received
MISSION_TIMEOUT_FACTOR = 5.0  # 500% safety margin for mission timeouts
DEFAULT_NAMESPACE_HOLD_TIME = 300.0  # 5 minutes default hold time if no mission data


class RoverState(Enum):
    """Rover connection state from dock's perspective."""
    CONNECTING = "CONNECTING"
    IDENTIFIED = "IDENTIFIED"
    VERIFIED = "VERIFIED"
    ACTIVE = "ACTIVE"
    DISCONNECTED = "DISCONNECTED"


@dataclass
class ConnectedRover:
    """State for a connected rover."""
    module_id: str
    writer: asyncio.StreamWriter
    reader: asyncio.StreamReader
    state: RoverState = RoverState.CONNECTING
    module_type: str = "unknown"
    firmware: str = "unknown"
    battery_level: float = 1.0
    last_heartbeat: float = field(default_factory=time.time)
    last_odom: Optional[Odometry] = None
    last_scan: Optional[LaserScan] = None
    # Position tracking
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    # Mission tracking for namespace lifecycle
    current_mission_id: Optional[str] = None
    mission_estimated_path_m: float = 0.0  # Estimated path distance in meters
    mission_max_speed_mps: float = 0.3  # Max speed from config (default 0.3 m/s)


# -----------------------------------------------------------------------------
# RoverBridge Node
# -----------------------------------------------------------------------------
class RoverBridge(Node):
    """
    ROS2 node that bridges lightweight Rust rovers to the ROS2 ecosystem.

    For each connected rover, creates:
    - /<rover_id>/scan (sensor_msgs/LaserScan) - LiDAR data
    - /<rover_id>/odom (nav_msgs/Odometry) - Wheel odometry
    - /<rover_id>/cmd_vel (geometry_msgs/Twist) subscriber - velocity commands

    Also publishes TF transforms for each rover.
    """

    def __init__(self):
        super().__init__('rover_bridge')

        # Parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('dock_id', '')
        self.declare_parameter('coven_name', '')
        self.declare_parameter('data_base_dir', os.path.expanduser('~/Desktop/COVEN/Data'))

        self.port = self.get_parameter('port').value
        self.dock_id = self.get_parameter('dock_id').value or f"dock_{id(self) % 10000:04d}"
        self.coven_name = self.get_parameter('coven_name').value or get_coven_name()
        self.data_base_dir = self.get_parameter('data_base_dir').value

        # Session timestamp for data directory (YYYYMMDD.HHMM.SS format, 24h clock)
        self.session_start = datetime.now()
        self.session_timestamp = self.session_start.strftime('%Y%m%d.%H%M.%S')
        self.session_data_dir = os.path.join(
            self.data_base_dir, self.session_timestamp, self.coven_name
        )
        os.makedirs(self.session_data_dir, exist_ok=True)
        self.get_logger().info(f"Data directory: {self.session_data_dir}")

        # Connected rovers
        self.rovers: Dict[str, ConnectedRover] = {}
        self.rovers_lock = threading.Lock()

        # Per-rover publishers (created dynamically)
        self.scan_pubs: Dict[str, any] = {}
        self.odom_pubs: Dict[str, any] = {}
        self.cmd_vel_subs: Dict[str, any] = {}

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS profiles - use RELIABLE for compatibility with ros2 topic echo
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # COVEN protocol publisher (for dock integration)
        self.coven_pub = self.create_publisher(String, '/coven/bridge_events', 10)

        # Task auctioneer for mission dispatch
        self.auctioneer = TaskAuctioneer(
            dock_id=self.dock_id,
            dock_position=(0.0, 0.0),  # TODO: make configurable
            bid_timeout=2.0,
        )
        self.auctioneer.set_send_task_callback(self._send_task_from_auctioneer)
        self.auctioneer.set_mission_complete_callback(self._on_mission_complete)

        # Mission dispatch subscriber (for external mission requests)
        self.mission_sub = self.create_subscription(
            String, '/coven/mission_request', self._mission_request_callback, 10
        )

        # Auctioneer status publisher
        self.auctioneer_status_pub = self.create_publisher(
            String, '/coven/auctioneer_status', 10
        )

        # Publisher for notifying offline SLAM processor of new sensor batches
        self.sensor_batch_pub = self.create_publisher(
            String, '/coven/sensor_batch', 10
        )

        # Publisher for rover status (consumed by frontier_dispatcher)
        self.rover_status_pub = self.create_publisher(
            String, '/coven/rover_status', 10
        )

        # Message queues for thread-safe publishing
        import queue
        self.odom_queue: queue.Queue = queue.Queue()
        self.scan_queue: queue.Queue = queue.Queue()
        self.tf_queue: queue.Queue = queue.Queue()

        # Timer to process queues (runs in ROS2 thread)
        self.create_timer(0.01, self._process_publish_queues)  # 100 Hz

        # Heartbeat check timer
        self.create_timer(1.0, self._check_heartbeats)

        # Auctioneer dispatch timer - try to dispatch missions periodically
        self.create_timer(2.0, self._try_dispatch_missions)

        # Auctioneer status timer
        self.create_timer(5.0, self._publish_auctioneer_status)

        # Async event loop for TCP server
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.server_task: Optional[asyncio.Task] = None

        # Start TCP server in background thread
        self.tcp_thread = threading.Thread(target=self._run_tcp_server, daemon=True)
        self.tcp_thread.start()

        self.get_logger().info(
            f"RoverBridge started - Dock: {self.dock_id}, Coven: {self.coven_name}, Port: {self.port}"
        )

    def _run_tcp_server(self):
        """Run the async TCP server in a dedicated thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        try:
            self.loop.run_until_complete(self._start_server())
        except Exception as e:
            self.get_logger().error(f"TCP server error: {e}")
        finally:
            self.loop.close()

    async def _start_server(self):
        """Start the TCP server."""
        server = await asyncio.start_server(
            self._handle_client,
            '0.0.0.0',
            self.port
        )

        addr = server.sockets[0].getsockname()
        self.get_logger().info(f"TCP server listening on {addr}")

        async with server:
            await server.serve_forever()

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Handle a new rover connection.

        Supports three scenarios:
        1. New rover: Assign fresh witch name
        2. Returning rover (within timeout): "Welcome back <name>!"
        3. Returning rover (name reassigned): "No you aren't. You are <new_name>!"
        """
        addr = writer.get_extra_info('peername')
        self.get_logger().info(f"New connection from {addr}")

        # Create temporary rover entry - will be updated after IDENTIFY_REP
        rover = ConnectedRover(
            module_id="pending",
            writer=writer,
            reader=reader,
            state=RoverState.CONNECTING
        )

        try:
            # First, send a query to see if rover claims an existing identity
            # Format: IDENTIFY_REQ:dock_id:coven_name:suggested_name
            # For new connections, we send empty suggested_name to ask rover its identity
            await self._send_message(
                rover,
                f"IDENTIFY_REQ:{self.dock_id}:{self.coven_name}:"
            )

            # Process messages
            while True:
                try:
                    line = await asyncio.wait_for(reader.readline(), timeout=HEARTBEAT_TIMEOUT * 2)
                    if not line:
                        break

                    message = line.decode('utf-8').strip()
                    if message:
                        await self._process_message(rover, message)

                except asyncio.TimeoutError:
                    self.get_logger().warn(f"Timeout waiting for data from {rover.module_id}")
                    break

        except Exception as e:
            self.get_logger().error(f"Error handling {rover.module_id}: {e}")

        finally:
            await self._disconnect_rover(rover)
            writer.close()
            await writer.wait_closed()

    async def _send_message(self, rover: ConnectedRover, message: str):
        """Send a message to a rover."""
        try:
            rover.writer.write((message + "\n").encode('utf-8'))
            await rover.writer.drain()
            self.get_logger().debug(f"Sent to {rover.module_id}: {message[:100]}...")
        except Exception as e:
            self.get_logger().error(f"Failed to send to {rover.module_id}: {e}")

    async def _process_message(self, rover: ConnectedRover, message: str):
        """Process a message from a rover."""
        # Log all messages for debugging
        if message.startswith('{'):
            self.get_logger().info(f"JSON from {rover.module_id}: {message[:200]}...")
        else:
            self.get_logger().debug(f"Received from {rover.module_id}: {message[:100]}...")

        # Try JSON first (for complex messages)
        if message.startswith('{'):
            try:
                data = json.loads(message)
                await self._process_json_message(rover, data)
                return
            except json.JSONDecodeError:
                pass

        # Parse colon-delimited message
        parts = message.split(':')
        msg_type = parts[0] if parts else ""

        if msg_type == "IDENTIFY_REP":
            await self._handle_identify_rep(rover, parts)
        elif msg_type == "VERIFY_REP":
            await self._handle_verify_rep(rover, parts)
        elif msg_type == "HEARTBEAT":
            await self._handle_heartbeat(rover, parts)
        elif msg_type == "TASK_ACK":
            await self._handle_task_ack(rover, parts)
        elif msg_type == "TASK_START":
            await self._handle_task_start(rover, parts)
        elif msg_type == "TASK_COMPLETE":
            await self._handle_task_complete(rover, parts)
        else:
            self.get_logger().warn(f"Unknown message type: {msg_type}")

    async def _process_json_message(self, rover: ConnectedRover, data: dict):
        """Process a JSON message (scan, odom, or batch data)."""
        if "DataBatch" in data:
            # Batch upload from rover returning to dock
            # Format: {"DataBatch": {"module_id": "...", "mission_id": "...", "batch": {...}}}
            batch_wrapper = data["DataBatch"]
            mission_id = batch_wrapper.get("mission_id", "unknown")
            batch_data = batch_wrapper.get("batch", {})
            await self._handle_data_batch(rover, mission_id, batch_data)
        elif "ScanData" in data:
            # Rust serde format: {"ScanData": {"module_id": "...", "scan": {...}}}
            # Legacy real-time streaming (mock/debug mode only)
            scan_wrapper = data["ScanData"]
            scan_data = scan_wrapper.get("scan", scan_wrapper)
            await self._handle_scan_data(rover, scan_data)
        elif "OdomData" in data:
            # Rust serde format: {"OdomData": {"module_id": "...", "odom": {...}}}
            # Legacy real-time streaming (mock/debug mode only)
            odom_wrapper = data["OdomData"]
            odom_data = odom_wrapper.get("odom", odom_wrapper)
            await self._handle_odom_data(rover, odom_data)
        else:
            self.get_logger().warn(f"Unknown JSON message: {list(data.keys())}")

    async def _handle_identify_rep(self, rover: ConnectedRover, parts: List[str]):
        """Handle IDENTIFY_REP from rover.

        Implements the three reconnection scenarios:
        1. New rover (claims "new_witch"): Assign fresh witch name
        2. Returning rover (known name, within timeout): "Welcome back!"
        3. Returning rover (name was reassigned): "No you aren't. You are <new_name>!"
        """
        # Format: IDENTIFY_REP:claimed_name:module_type:firmware:battery:status
        if len(parts) < 6:
            self.get_logger().error(f"Malformed IDENTIFY_REP (need 6 parts): {parts}")
            return

        claimed_name = parts[1]  # What the rover claims to be
        rover.module_type = parts[2]
        rover.firmware = parts[3]
        # Battery comes as percentage (0-100) from Rust, convert to 0.0-1.0
        rover.battery_level = float(parts[4]) / 100.0 if len(parts) > 4 else 1.0
        rover_status = parts[5]

        if rover_status != "OK":
            self.get_logger().warning(
                f"Rover reported status '{rover_status}' during identification"
            )

        # Determine the rover's identity based on three scenarios
        old_id = rover.module_id
        final_name = None
        welcome_message = None

        if claimed_name == "new_witch" or not claimed_name:
            # Scenario 1: Brand new rover - assign a fresh witch name
            final_name = get_witch_name()
            welcome_message = f"I'm gonna call you {final_name}!"
            self.get_logger().info(
                f"New rover connected - assigning name: {final_name}"
            )

        elif is_witch_known(claimed_name):
            # Scenario 2: Returning rover within timeout window - welcome back!
            status = get_witch_status(claimed_name)
            if status == "missing":
                # Rover was marked as missing but within timeout - reabsorb
                final_name = get_witch_name(returning_name=claimed_name)
                if final_name == claimed_name:
                    welcome_message = f"Welcome back {final_name}!"
                    self.get_logger().info(
                        f"Returning rover {final_name} reconnected - welcome back!"
                    )
                else:
                    # This shouldn't happen if is_witch_known returned True, but handle it
                    welcome_message = f"No you aren't. You are {final_name}!"
                    self.get_logger().info(
                        f"Rover claimed {claimed_name} but assigned {final_name}"
                    )
            elif status == "active":
                # Name is currently active - this is a duplicate connection attempt
                # or the old connection didn't clean up properly
                self.get_logger().warning(
                    f"Rover claims to be {claimed_name} but that name is already active!"
                )
                # Force assign a new name
                final_name = get_witch_name()
                welcome_message = f"No you aren't. You are {final_name}!"
            else:
                # Status is "available" - name was released, assign fresh
                final_name = get_witch_name()
                welcome_message = f"No you aren't. You are {final_name}!"
                self.get_logger().info(
                    f"Rover claimed {claimed_name} (released), now {final_name}"
                )
        else:
            # Scenario 3: Rover claims name we don't recognize OR name was released
            # The rover might have been gone too long (past timeout)
            final_name = get_witch_name()
            welcome_message = f"No you aren't. You are {final_name}!"
            self.get_logger().info(
                f"Rover claimed unknown/expired name '{claimed_name}', assigned {final_name}"
            )

        # Update rover with final assigned name
        rover.module_id = final_name
        rover.state = RoverState.IDENTIFIED

        # Mark the name as active
        mark_witch_active(final_name)

        self.get_logger().info(
            f"{rover.module_id} reporting for duty! ({rover.module_type}, fw {rover.firmware}, battery {rover.battery_level*100:.0f}%)"
        )

        # Register rover with assigned name
        with self.rovers_lock:
            if old_id in self.rovers and old_id != final_name:
                del self.rovers[old_id]
            self.rovers[rover.module_id] = rover

        # Create ROS2 publishers/subscribers for this rover
        self._setup_rover_topics(rover.module_id)

        # Send identity confirmation back to rover with our assigned name
        # Format: IDENTIFY_ACK:dock_id:assigned_name:message
        await self._send_message(
            rover,
            f"IDENTIFY_ACK:{self.dock_id}:{final_name}:{welcome_message}"
        )

        # Send VERIFY_REQ
        await self._send_message(rover, f"VERIFY_REQ:{self.dock_id}:{rover.module_id}")

    async def _handle_verify_rep(self, rover: ConnectedRover, parts: List[str]):
        """Handle VERIFY_REP from rover."""
        # Format: VERIFY_REP:module_id:success:failed_checks:note
        if len(parts) < 3:
            return

        success = parts[2].lower() == "true"

        if success:
            rover.state = RoverState.VERIFIED
            self.get_logger().info(f"Rover {rover.module_id} verified successfully")

            # Register with auctioneer
            # Default to LIDAR payload - could parse from module_type
            payload = PayloadType.LIDAR
            if "spectro" in rover.module_type.lower():
                payload = PayloadType.SPECTROMETER
            elif "drill" in rover.module_type.lower():
                payload = PayloadType.DRILL
            elif "cargo" in rover.module_type.lower():
                payload = PayloadType.CARGO
            elif "camera" in rover.module_type.lower():
                payload = PayloadType.CAMERA

            self.auctioneer.register_rover(
                module_id=rover.module_id,
                payload=payload,
                battery_pct=rover.battery_level * 100.0,
                position=(rover.x, rover.y),
                heading=rover.theta,
            )
            self.auctioneer.update_rover_status(
                module_id=rover.module_id,
                status=AuctionRoverStatus.IDLE,
            )

            # Publish event
            self._publish_event("ROVER_VERIFIED", rover.module_id, {
                "payload": payload.value,
            })

            # Transition to active
            rover.state = RoverState.ACTIVE
        else:
            failed_checks = parts[3] if len(parts) > 3 else "unknown"
            self.get_logger().warn(f"Rover {rover.module_id} verification failed: {failed_checks}")

    async def _handle_heartbeat(self, rover: ConnectedRover, parts: List[str]):
        """Handle HEARTBEAT from rover."""
        # Format: HEARTBEAT:module_id:battery_pct:mission_status:x:y:theta
        if len(parts) < 7:
            rover.last_heartbeat = time.time()
            return

        rover.battery_level = float(parts[2]) / 100.0
        mission_status = parts[3]  # "IDLE", "ACTIVE", "STARTUP"
        rover.x = float(parts[4])
        rover.y = float(parts[5])
        rover.theta = float(parts[6])
        rover.last_heartbeat = time.time()

        # Update auctioneer with rover status
        auction_status = AuctionRoverStatus.IDLE
        if mission_status == "ACTIVE":
            auction_status = AuctionRoverStatus.ACTIVE
        elif mission_status == "STARTUP":
            auction_status = AuctionRoverStatus.UNKNOWN

        self.auctioneer.update_rover_status(
            module_id=rover.module_id,
            status=auction_status,
            battery_pct=rover.battery_level * 100.0,
            position=(rover.x, rover.y),
            heading=rover.theta,
        )

        # Publish rover status for frontier_dispatcher
        self._publish_rover_status(rover, mission_status)

        # Publish TF
        self._publish_rover_tf(rover)

    async def _handle_scan_data(self, rover: ConnectedRover, scan_data: dict):
        """Handle scan data from rover, publish as LaserScan."""
        if rover.module_id not in self.scan_pubs:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f"{rover.module_id}/laser_frame"

        scan.angle_min = float(scan_data.get("angle_min", -math.pi))
        scan.angle_max = float(scan_data.get("angle_max", math.pi))
        scan.angle_increment = float(scan_data.get("angle_increment", 0.01))
        scan.range_min = float(scan_data.get("range_min", 0.12))
        scan.range_max = float(scan_data.get("range_max", 10.0))

        # Convert ranges from mm to meters
        ranges_mm = scan_data.get("ranges_mm", [])
        scan.ranges = [
            float(r) / 1000.0 if r > 0 else float('inf')
            for r in ranges_mm
        ]

        # Calculate time increment based on scan rate
        if len(scan.ranges) > 0:
            scan.time_increment = (1.0 / 6.0) / len(scan.ranges)  # ~6 Hz scan rate
            scan.scan_time = 1.0 / 6.0

        # Schedule publish on ROS2 thread
        self._publish_scan_threadsafe(rover.module_id, scan)
        rover.last_scan = scan

    async def _handle_odom_data(self, rover: ConnectedRover, odom_data: dict):
        """Handle odometry data from rover, publish as Odometry."""
        if rover.module_id not in self.odom_pubs:
            self.get_logger().warn(f"No odom pub for {rover.module_id}, available: {list(self.odom_pubs.keys())}")
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = f"{rover.module_id}/base_link"

        # Position
        odom.pose.pose.position.x = float(odom_data.get("x", 0.0))
        odom.pose.pose.position.y = float(odom_data.get("y", 0.0))
        odom.pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        theta = float(odom_data.get("theta", 0.0))
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = float(odom_data.get("v_linear", 0.0))
        odom.twist.twist.angular.z = float(odom_data.get("v_angular", 0.0))

        # Schedule publish on ROS2 thread
        self._publish_odom_threadsafe(rover.module_id, odom)
        rover.last_odom = odom

        # Update rover position
        rover.x = odom.pose.pose.position.x
        rover.y = odom.pose.pose.position.y
        rover.theta = theta

        # Publish TF
        self._publish_rover_tf(rover)

    async def _handle_data_batch(self, rover: ConnectedRover, mission_id: str, batch: dict):
        """
        Handle batch data upload from rover returning to dock.

        The rover is 'dumb' - it just collects raw encoder ticks and LiDAR ranges.
        The dock (this node) converts the raw data to odometry and publishes it
        for SLAM processing.

        Batch format:
        {
            "mission_start": <unix_timestamp>,
            "wheel_radius_mm": <int>,
            "wheel_base_mm": <int>,
            "ticks_per_rev": <int>,
            "lidar_angle_min": <float>,
            "lidar_angle_max": <float>,
            "lidar_num_rays": <int>,
            "samples": [
                {
                    "timestamp": <float>,  # seconds since mission_start
                    "left_ticks": <int>,   # delta ticks since last sample
                    "right_ticks": <int>,
                    "lidar_ranges_mm": [<int>, ...]  # 0 = no return
                },
                ...
            ]
        }
        """
        samples = batch.get("samples", [])
        num_samples = len(samples)

        self.get_logger().info(
            f"Received data batch from {rover.module_id}: mission={mission_id}, "
            f"{num_samples} samples"
        )

        # === BATCH VALIDATION ===
        # Reject empty or corrupted batches
        MIN_SAMPLES = 10  # Minimum samples for valid batch

        if num_samples < MIN_SAMPLES:
            self.get_logger().error(
                f"REJECTING batch from {rover.module_id}: only {num_samples} samples "
                f"(minimum {MIN_SAMPLES} required). Batch may be corrupted or mission too short."
            )
            self._publish_event("DATA_BATCH_REJECTED", rover.module_id, {
                "mission_id": mission_id,
                "num_samples": num_samples,
                "reason": f"insufficient_samples (need {MIN_SAMPLES}, got {num_samples})",
            })
            return

        # Validate required fields
        required_fields = ["wheel_radius_mm", "wheel_base_mm", "ticks_per_rev"]
        missing_fields = [f for f in required_fields if f not in batch or batch[f] <= 0]
        if missing_fields:
            self.get_logger().error(
                f"REJECTING batch from {rover.module_id}: missing or invalid fields: {missing_fields}"
            )
            self._publish_event("DATA_BATCH_REJECTED", rover.module_id, {
                "mission_id": mission_id,
                "reason": f"missing_fields: {missing_fields}",
            })
            return

        # Extract robot configuration
        wheel_radius = batch.get("wheel_radius_mm", 80) / 1000.0  # to meters
        wheel_base = batch.get("wheel_base_mm", 298) / 1000.0
        ticks_per_rev = batch.get("ticks_per_rev", 1440)
        mission_start = batch.get("mission_start", 0.0)

        # LiDAR config
        lidar_angle_min = batch.get("lidar_angle_min", -math.pi)
        lidar_angle_max = batch.get("lidar_angle_max", math.pi)
        lidar_num_rays = batch.get("lidar_num_rays", 360)
        lidar_angle_increment = (lidar_angle_max - lidar_angle_min) / max(lidar_num_rays - 1, 1)

        # Calculate meters per tick
        wheel_circumference = 2.0 * math.pi * wheel_radius
        meters_per_tick = wheel_circumference / ticks_per_rev

        # Process samples - convert raw ticks to odometry
        x, y, theta = 0.0, 0.0, 0.0
        # samples already extracted during validation above

        for sample in samples:
            ts = sample.get("timestamp", 0.0)
            left_ticks = sample.get("left_ticks", 0)
            right_ticks = sample.get("right_ticks", 0)
            lidar_ranges_mm = sample.get("lidar_ranges_mm", [])

            # Convert ticks to distance
            dist_left = left_ticks * meters_per_tick
            dist_right = right_ticks * meters_per_tick

            # Differential drive kinematics
            dist_center = (dist_left + dist_right) / 2.0
            delta_theta = (dist_right - dist_left) / wheel_base

            # Update pose using midpoint integration
            theta_mid = theta + delta_theta / 2.0
            x += dist_center * math.cos(theta_mid)
            y += dist_center * math.sin(theta_mid)
            theta += delta_theta

            # Normalize theta to [-pi, pi]
            while theta > math.pi:
                theta -= 2.0 * math.pi
            while theta < -math.pi:
                theta += 2.0 * math.pi

            # Create and publish Odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = f"{rover.module_id}/base_link"

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)

            self._publish_odom_threadsafe(rover.module_id, odom)

            # Create and publish LaserScan message if we have LiDAR data
            if lidar_ranges_mm:
                scan = LaserScan()
                scan.header.stamp = odom.header.stamp
                scan.header.frame_id = f"{rover.module_id}/laser_frame"

                scan.angle_min = lidar_angle_min
                scan.angle_max = lidar_angle_max
                scan.angle_increment = lidar_angle_increment
                scan.range_min = 0.12
                scan.range_max = 10.0

                # Convert mm to meters, 0 = inf (no return)
                scan.ranges = [
                    float(r) / 1000.0 if r > 0 else float('inf')
                    for r in lidar_ranges_mm
                ]

                self._publish_scan_threadsafe(rover.module_id, scan)

        # Update rover position from final sample
        rover.x = x
        rover.y = y
        rover.theta = theta

        # Save batch data to disk for offline SLAM processing
        # Retry up to 3 times on failure
        MAX_SAVE_RETRIES = 3
        saved_filename = None

        for attempt in range(1, MAX_SAVE_RETRIES + 1):
            saved_filename = self._save_batch_to_disk(
                rover, mission_id, batch, samples,
                wheel_radius, wheel_base, ticks_per_rev, meters_per_tick,
                lidar_angle_min, lidar_angle_max, lidar_angle_increment
            )
            if saved_filename:
                break
            self.get_logger().warning(
                f"Save attempt {attempt}/{MAX_SAVE_RETRIES} failed for batch from {rover.module_id}"
            )
            if attempt < MAX_SAVE_RETRIES:
                await asyncio.sleep(0.5)  # Brief delay before retry

        # Notify offline SLAM processor if we saved successfully
        if saved_filename:
            self._notify_slam_processor(
                rover.module_id, mission_id, saved_filename, len(samples)
            )
            # Publish success event
            self._publish_event("DATA_BATCH_RECEIVED", rover.module_id, {
                "mission_id": mission_id,
                "num_samples": len(samples),
                "final_x": x,
                "final_y": y,
                "final_theta": theta,
                "saved_file": saved_filename,
            })
        else:
            # All retries failed - publish failure event
            self.get_logger().error(
                f"FAILED to save batch from {rover.module_id} after {MAX_SAVE_RETRIES} attempts. "
                "Data lost - SLAM will not process this mission."
            )
            self._publish_event("DATA_BATCH_SAVE_FAILED", rover.module_id, {
                "mission_id": mission_id,
                "num_samples": len(samples),
                "reason": "file_save_failed_after_retries",
            })

        self.get_logger().info(
            f"Processed batch from {rover.module_id}: final pose x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}°"
        )

    def _save_batch_to_disk(
        self, rover: ConnectedRover, mission_id: str, batch: dict,
        samples: list, wheel_radius: float, wheel_base: float,
        ticks_per_rev: int, meters_per_tick: float,
        lidar_angle_min: float, lidar_angle_max: float, lidar_angle_increment: float
    ) -> Optional[str]:
        """
        Save processed batch data to disk in the COVEN directory structure.

        Format: ~/Desktop/COVEN/Data/{YYYYMMDD.HHMM.SS}/{coven}/{rover}/Scan.{D.HHMM.SS}.json
        Where D.HHMM.SS is elapsed time since session start when data was delivered.

        Returns the saved filename on success, None on failure.
        """
        # Create rover data directory
        rover_dir = os.path.join(self.session_data_dir, rover.module_id)
        os.makedirs(rover_dir, exist_ok=True)

        # Calculate elapsed time since session start (D.HHMM.SS format, 24h clock)
        elapsed = datetime.now() - self.session_start
        elapsed_days = elapsed.days
        elapsed_secs = elapsed.seconds
        elapsed_hours = elapsed_secs // 3600
        elapsed_mins = (elapsed_secs % 3600) // 60
        elapsed_sec = elapsed_secs % 60
        elapsed_str = f"{elapsed_days}.{elapsed_hours:02d}{elapsed_mins:02d}.{elapsed_sec:02d}"

        # Filename format: Scan.{D.HHMM.SS}.json
        filename = os.path.join(rover_dir, f"Scan.{elapsed_str}.json")

        # Convert samples to frames format expected by offline_slam_processor
        frames = []
        x, y, theta = 0.0, 0.0, 0.0

        for sample in samples:
            ts = sample.get("timestamp", 0.0)
            left_ticks = sample.get("left_ticks", 0)
            right_ticks = sample.get("right_ticks", 0)
            lidar_ranges_mm = sample.get("lidar_ranges_mm", [])

            # Convert ticks to distance
            dist_left = left_ticks * meters_per_tick
            dist_right = right_ticks * meters_per_tick

            # Differential drive kinematics
            dist_center = (dist_left + dist_right) / 2.0
            delta_theta = (dist_right - dist_left) / wheel_base

            # Update pose
            theta_mid = theta + delta_theta / 2.0
            x += dist_center * math.cos(theta_mid)
            y += dist_center * math.sin(theta_mid)
            theta += delta_theta

            # Normalize theta
            while theta > math.pi:
                theta -= 2.0 * math.pi
            while theta < -math.pi:
                theta += 2.0 * math.pi

            # Convert LiDAR to meters
            scan_ranges = [
                float(r) / 1000.0 if r > 0 else float('inf')
                for r in lidar_ranges_mm
            ]

            frame = {
                "timestamp": ts,
                "odom_x": x,
                "odom_y": y,
                "odom_theta": theta,
                "odom_vx": 0.0,
                "odom_vy": 0.0,
                "odom_vtheta": 0.0,
                "scan_angle_min": lidar_angle_min,
                "scan_angle_max": lidar_angle_max,
                "scan_angle_increment": lidar_angle_increment,
                "scan_ranges": scan_ranges,
            }
            frames.append(frame)

        # Build mission data structure (matches offline_slam_processor expected format)
        # Calculate mission duration from first/last sample timestamps
        mission_start_ts = batch.get("mission_start", 0.0)
        first_ts = samples[0].get("timestamp", 0.0) if samples else 0.0
        last_ts = samples[-1].get("timestamp", 0.0) if samples else 0.0
        mission_duration = last_ts - first_ts

        mission_data = {
            "mission_id": mission_id,
            "module_id": rover.module_id,
            "start_time": mission_start_ts,
            "end_time": mission_start_ts + mission_duration,
            "initial_x": 0.0,  # Rover starts at origin in its local frame
            "initial_y": 0.0,
            "initial_theta": 0.0,
            "frames": frames,
            "metadata": {
                "wheel_radius_m": wheel_radius,
                "wheel_base_m": wheel_base,
                "ticks_per_rev": ticks_per_rev,
                "dock_id": self.dock_id,
                "coven_name": self.coven_name,
            }
        }

        try:
            with open(filename, 'w') as f:
                json.dump(mission_data, f, indent=2)
            self.get_logger().info(f"Saved batch data to {filename}")
            return filename
        except Exception as e:
            self.get_logger().error(f"Failed to save batch data: {e}")
            return None

    def _notify_slam_processor(
        self, module_id: str, mission_id: str, filename: str, frame_count: int
    ):
        """
        Notify the offline SLAM processor that new sensor data is available.

        Publishes to /coven/sensor_batch for processing by the SLAM pipeline.
        """
        msg = String()
        msg.data = json.dumps({
            "type": "transfer_complete",
            "filename": filename,
            "mission_id": mission_id,
            "module_id": module_id,
            "frame_count": frame_count,
        })
        self.sensor_batch_pub.publish(msg)
        self.get_logger().info(
            f"Notified SLAM processor: {mission_id} ({frame_count} frames)"
        )

    async def _handle_task_ack(self, rover: ConnectedRover, parts: List[str]):
        """Handle TASK_ACK from rover."""
        self._publish_event("TASK_ACK", rover.module_id, {"parts": parts})

    async def _handle_task_start(self, rover: ConnectedRover, parts: List[str]):
        """Handle TASK_START from rover."""
        self._publish_event("TASK_START", rover.module_id, {"parts": parts})

    async def _handle_task_complete(self, rover: ConnectedRover, parts: List[str]):
        """Handle TASK_COMPLETE from rover."""
        # Format: TASK_COMPLETE:module_id:task_id:success:map_data:coverage:duration
        task_id = parts[2] if len(parts) > 2 else "unknown"
        success = parts[3].lower() == "true" if len(parts) > 3 else False
        duration = float(parts[6]) if len(parts) > 6 else 0.0

        # Clear mission tracking - rover is back at dock
        rover.current_mission_id = None
        rover.mission_estimated_path_m = 0.0

        # Notify auctioneer
        self.auctioneer.handle_task_complete(
            module_id=rover.module_id,
            task_id=task_id,
            success=success,
            duration=duration,
        )

        self._publish_event("TASK_COMPLETE", rover.module_id, {
            "parts": parts,
            "task_id": task_id,
            "success": success,
            "duration": duration,
        })

    def _setup_rover_topics(self, module_id: str):
        """Create ROS2 publishers and subscribers for a rover."""
        # LaserScan publisher
        self.scan_pubs[module_id] = self.create_publisher(
            LaserScan,
            f'/{module_id}/scan',
            self.sensor_qos
        )

        # Odometry publisher
        self.odom_pubs[module_id] = self.create_publisher(
            Odometry,
            f'/{module_id}/odom',
            self.sensor_qos
        )

        # cmd_vel subscriber
        self.cmd_vel_subs[module_id] = self.create_subscription(
            Twist,
            f'/{module_id}/cmd_vel',
            lambda msg, mid=module_id: self._cmd_vel_callback(mid, msg),
            10
        )

        self.get_logger().info(f"Created topics for rover {module_id}")

    def _teardown_rover_topics(self, module_id: str):
        """Remove ROS2 publishers and subscribers for a rover."""
        if module_id in self.scan_pubs:
            self.destroy_publisher(self.scan_pubs.pop(module_id))
        if module_id in self.odom_pubs:
            self.destroy_publisher(self.odom_pubs.pop(module_id))
        if module_id in self.cmd_vel_subs:
            self.destroy_subscription(self.cmd_vel_subs.pop(module_id))

    def _process_publish_queues(self):
        """Process message queues - called from ROS2 timer (main thread)."""
        # Process odom queue
        while not self.odom_queue.empty():
            try:
                module_id, odom = self.odom_queue.get_nowait()
                if module_id in self.odom_pubs:
                    self.odom_pubs[module_id].publish(odom)
            except:
                break

        # Process scan queue
        while not self.scan_queue.empty():
            try:
                module_id, scan = self.scan_queue.get_nowait()
                if module_id in self.scan_pubs:
                    self.scan_pubs[module_id].publish(scan)
            except:
                break

        # Process TF queue
        while not self.tf_queue.empty():
            try:
                transform = self.tf_queue.get_nowait()
                self.tf_broadcaster.sendTransform(transform)
            except:
                break

    def _publish_odom_threadsafe(self, module_id: str, odom: Odometry):
        """Queue odom for thread-safe publishing."""
        self.odom_queue.put((module_id, odom))

    def _publish_scan_threadsafe(self, module_id: str, scan: LaserScan):
        """Queue scan for thread-safe publishing."""
        self.scan_queue.put((module_id, scan))

    def _publish_tf_threadsafe(self, transform: TransformStamped):
        """Queue TF for thread-safe publishing."""
        self.tf_queue.put(transform)

    def _cmd_vel_callback(self, module_id: str, msg: Twist):
        """Handle cmd_vel from ROS2, forward to rover."""
        with self.rovers_lock:
            rover = self.rovers.get(module_id)
            if not rover or rover.state != RoverState.ACTIVE:
                return

        # Format: CMD_VEL:linear:angular
        cmd = f"CMD_VEL:{msg.linear.x:.4f}:{msg.angular.z:.4f}"

        # Schedule send in the async loop
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._send_message(rover, cmd),
                self.loop
            )

    def _publish_rover_tf(self, rover: ConnectedRover):
        """Publish TF transform for a rover."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = f"{rover.module_id}/base_link"

        t.transform.translation.x = rover.x
        t.transform.translation.y = rover.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(rover.theta / 2.0)
        t.transform.rotation.w = math.cos(rover.theta / 2.0)

        # Schedule TF broadcast on ROS2 thread
        self._publish_tf_threadsafe(t)

    def _publish_rover_status(self, rover: ConnectedRover, mission_status: str):
        """Publish rover status for frontier_dispatcher.

        Format matches what frontier_dispatcher._rover_status_callback expects:
        {
            "module_id": str,
            "state": "idle" | "deployed" | "returning" | "charging" | "error",
            "position": {"x": float, "y": float},
            "battery_level": float (0-1),
            "current_mission": str or None
        }
        """
        # Map mission_status from rover to dispatcher's expected format
        state_map = {
            "IDLE": "idle",
            "ACTIVE": "deployed",
            "STARTUP": "idle",
            "RETURNING": "returning",
        }
        state = state_map.get(mission_status, "idle")

        status_msg = {
            "module_id": rover.module_id,
            "state": state,
            "position": {"x": rover.x, "y": rover.y},
            "battery_level": rover.battery_level,
            "current_mission": getattr(rover, 'current_mission', None),
        }

        msg = String()
        msg.data = json.dumps(status_msg)
        self.rover_status_pub.publish(msg)

    def _publish_event(self, event_type: str, module_id: str, extra: dict = None):
        """Publish a COVEN bridge event."""
        event = {
            "type": event_type,
            "module_id": module_id,
            "timestamp": time.time(),
            "dock_id": self.dock_id
        }
        if extra:
            event.update(extra)

        msg = String()
        msg.data = json.dumps(event)
        self.coven_pub.publish(msg)

    def _check_heartbeats(self):
        """Check for timed-out rovers."""
        now = time.time()
        disconnected = []

        with self.rovers_lock:
            for module_id, rover in self.rovers.items():
                if now - rover.last_heartbeat > HEARTBEAT_TIMEOUT:
                    disconnected.append(module_id)

        for module_id in disconnected:
            self.get_logger().warn(f"Rover {module_id} heartbeat timeout")
            # Mark as disconnected but don't remove yet
            with self.rovers_lock:
                if module_id in self.rovers:
                    self.rovers[module_id].state = RoverState.DISCONNECTED

    async def _disconnect_rover(self, rover: ConnectedRover):
        """Clean up a disconnected rover.

        Marks the rover's namespace as "missing" rather than immediately releasing it.
        This allows the rover to reclaim its name if it reconnects within the timeout.

        Timeout calculation:
        - If rover had an active mission: 500% of estimated mission time
        - Otherwise: DEFAULT_NAMESPACE_HOLD_TIME (5 minutes)
        """
        self.get_logger().info(f"Rover {rover.module_id} disconnected")

        # Calculate namespace hold timeout
        if rover.mission_estimated_path_m > 0 and rover.mission_max_speed_mps > 0:
            # Calculate timeout as 500% of estimated mission time
            # Time = distance / speed, then multiply by MISSION_TIMEOUT_FACTOR
            estimated_time = rover.mission_estimated_path_m / rover.mission_max_speed_mps
            namespace_timeout = estimated_time * MISSION_TIMEOUT_FACTOR
            self.get_logger().info(
                f"Holding namespace '{rover.module_id}' for {namespace_timeout:.0f}s "
                f"(500% of {estimated_time:.0f}s estimated mission time)"
            )
        else:
            # No mission data - use default hold time
            namespace_timeout = DEFAULT_NAMESPACE_HOLD_TIME
            self.get_logger().info(
                f"Holding namespace '{rover.module_id}' for {namespace_timeout:.0f}s (default)"
            )

        # Mark the witch name as missing (not available for reassignment yet)
        mark_witch_missing(rover.module_id, namespace_timeout)

        with self.rovers_lock:
            if rover.module_id in self.rovers:
                del self.rovers[rover.module_id]

        self._teardown_rover_topics(rover.module_id)
        self._publish_event("ROVER_DISCONNECTED", rover.module_id, {
            "namespace_hold_timeout": namespace_timeout,
        })

    def get_connected_rovers(self) -> List[str]:
        """Get list of connected rover IDs."""
        with self.rovers_lock:
            return [
                mid for mid, r in self.rovers.items()
                if r.state == RoverState.ACTIVE
            ]

    # -------------------------------------------------------------------------
    # Auctioneer Integration
    # -------------------------------------------------------------------------

    async def _send_task_from_auctioneer(self, module_id: str, task_req: dict):
        """Callback from auctioneer to send TASK_REQ to a rover."""
        with self.rovers_lock:
            rover = self.rovers.get(module_id)
            if not rover or rover.state != RoverState.ACTIVE:
                self.get_logger().error(f"Cannot send task to {module_id}: not active")
                return

        # Extract mission info for namespace lifecycle tracking
        task_id = task_req.get("task_id", "unknown")
        waypoints = task_req.get("waypoints", [])

        # Calculate estimated path distance from waypoints
        estimated_path = 0.0
        if waypoints:
            # Start from rover's current position
            prev_x, prev_y = rover.x, rover.y
            for wp in waypoints:
                if isinstance(wp, dict):
                    wx, wy = wp.get("x", 0), wp.get("y", 0)
                elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                    wx, wy = wp[0], wp[1]
                else:
                    continue
                estimated_path += math.sqrt((wx - prev_x)**2 + (wy - prev_y)**2)
                prev_x, prev_y = wx, wy
            # Add return to dock (origin)
            estimated_path += math.sqrt(prev_x**2 + prev_y**2)

        # Update rover's mission tracking
        rover.current_mission_id = task_id
        rover.mission_estimated_path_m = estimated_path
        # max_speed could come from config, for now use default
        rover.mission_max_speed_mps = 0.3

        self.get_logger().info(
            f"Mission {task_id}: estimated path {estimated_path:.1f}m, "
            f"timeout will be {(estimated_path / rover.mission_max_speed_mps) * MISSION_TIMEOUT_FACTOR:.0f}s"
        )

        # Format task as JSON (matches what rover expects)
        task_json = json.dumps(task_req)
        cmd = f"TASK_REQ:{self.dock_id}:{task_id}:{task_json}"

        await self._send_message(rover, cmd)
        self.get_logger().info(f"Sent TASK_REQ to {module_id}: {task_id}")

    def _on_mission_complete(self, mission: Mission, success: bool):
        """Callback from auctioneer when mission completes."""
        self.get_logger().info(
            f"Mission {mission.mission_id} complete: "
            f"{'SUCCESS' if success else 'FAILED'}"
        )
        # Could trigger next mission queue here, or other logic

    def _try_dispatch_missions(self):
        """Timer callback to try dispatching missions from queue."""
        if not self.auctioneer.mission_queue:
            return

        idle_count = len(self.auctioneer.get_idle_rovers())
        if idle_count == 0:
            return

        # Run the async dispatch in the TCP thread's event loop
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.auctioneer.try_dispatch(),
                self.loop
            )

    def _publish_auctioneer_status(self):
        """Publish auctioneer status for monitoring."""
        status = self.auctioneer.get_status()
        msg = String()
        msg.data = json.dumps(status)
        self.auctioneer_status_pub.publish(msg)

    def _mission_request_callback(self, msg: String):
        """Handle external mission requests."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid mission request JSON")
            return

        # Expected format (accepts both styles):
        # {
        #   "type" or "task_type": "explore" | "spectral" | "sample" | "deliver" | "survey",
        #   "waypoints": [[x, y], ...] or [{"x": x, "y": y}, ...],
        #   "priority": 0  (optional)
        # }
        task_type_str = data.get("type", data.get("task_type", "explore")).lower()
        waypoints = data.get("waypoints", [])
        priority = data.get("priority", 0)

        if not waypoints:
            self.get_logger().error("Mission request missing waypoints")
            return

        # Convert waypoints to tuples (handle both [x,y] and {"x":x,"y":y} formats)
        wp_tuples = []
        for wp in waypoints:
            if isinstance(wp, dict):
                wp_tuples.append((wp["x"], wp["y"]))
            else:
                wp_tuples.append((wp[0], wp[1]))

        # Map string to TaskType
        task_type_map = {
            "explore": TaskType.EXPLORE,
            "spectral": TaskType.SPECTRAL,
            "sample": TaskType.SAMPLE,
            "deliver": TaskType.DELIVER,
            "survey": TaskType.SURVEY,
        }
        task_type = task_type_map.get(task_type_str, TaskType.EXPLORE)

        # Create and queue mission
        import uuid
        mission = Mission(
            mission_id=f"{task_type_str}_{uuid.uuid4().hex[:8]}",
            task_type=task_type,
            waypoints=wp_tuples,
            dock_return=(0.0, 0.0),  # Return to dock origin
            priority=priority,
        )
        self.auctioneer.queue_mission(mission)

        self.get_logger().info(
            f"Queued mission {mission.mission_id}: "
            f"{len(wp_tuples)} waypoints, priority={priority}"
        )

    def queue_exploration_mission(self, waypoints: List[tuple], priority: int = 0):
        """Convenience method to queue an exploration mission."""
        return self.auctioneer.create_exploration_mission(waypoints, priority)

    def send_task_to_rover(self, module_id: str, task_id: str, task_data: dict):
        """Send a task request to a specific rover."""
        with self.rovers_lock:
            rover = self.rovers.get(module_id)
            if not rover or rover.state != RoverState.ACTIVE:
                self.get_logger().error(f"Cannot send task to {module_id}: not active")
                return False

        # Format task as JSON
        task_json = json.dumps(task_data)
        cmd = f"TASK_REQ:{self.dock_id}:{task_id}:{task_json}"

        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._send_message(rover, cmd),
                self.loop
            )
            return True
        return False


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    node = RoverBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
