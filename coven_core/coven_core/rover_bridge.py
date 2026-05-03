# SPDX-License-Identifier: MIT
"""
rover_bridge.py — COVEN Rover Bridge

Bridges Rust rovers to the ROS2 ecosystem via serial UART.
Handles the COVEN handshake protocol and translates between:
- Rover sensor data (UART) -> ROS2 topics (LaserScan, Odometry)
- ROS2 cmd_vel -> UART velocity commands to rover

This node runs on the dock and manages the serial connection to a
rover that physically docks via USB (MVP) or the COVEN 9-pin
connector (production). NO wireless.

Protocol handling lives in bridge_protocol.py; batch processing in
bridge_data.py; per-rover topic management in bridge_topics.py.

Author: Alexander Shultis
Date: January 2025
"""

import glob
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Dict, Optional, List

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from coven_core.common import (
    get_coven_name, mark_witch_missing,
)
from coven_core.task_auctioneer import (
    TaskAuctioneer, Mission, TaskType,
)
from coven_core.bridge_topics import TopicManager
from coven_core.bridge_protocol import ProtocolHandler
from coven_core.bridge_data import DataBatchProcessor
from coven_core.frame_codec import (
    FrameParser,
    decode_message,
    encode_identify_request,
    encode_verify_ok,
    encode_verify_fail,
    encode_task_request,
    encode_cmd_vel,
)


# Configuration
DEFAULT_SERIAL_PORT = 'auto'  # Auto-detect: scans /dev/ttyACM* and /dev/ttyUSB*
DEFAULT_BAUD_RATE = 115200
HEARTBEAT_TIMEOUT = 30.0
CMD_VEL_TIMEOUT = 0.5
MISSION_TIMEOUT_FACTOR = 5.0
DEFAULT_NAMESPACE_HOLD_TIME = 300.0
SERIAL_RETRY_DELAY = 2.0  # seconds between serial reconnect attempts


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
    state: RoverState = RoverState.CONNECTING
    module_type: str = "unknown"
    firmware: str = "unknown"
    battery_level: float = 1.0
    last_heartbeat: float = field(default_factory=time.time)
    last_odom: Optional[Odometry] = None
    last_scan: Optional[LaserScan] = None
    capabilities: int = 0  # Bitmask from frame_codec CAP_* constants
    # Position tracking
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    # Mission tracking for namespace lifecycle
    current_mission_id: Optional[str] = None
    mission_estimated_path_m: float = 0.0
    mission_max_speed_mps: float = 0.3


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
        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('baud_rate', DEFAULT_BAUD_RATE)
        self.declare_parameter('dock_id', '')
        self.declare_parameter('coven_name', '')
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('data_base_dir', os.path.expanduser('~/Desktop/COVEN/Data'))

        self.serial_port_path = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.dock_id = self.get_parameter('dock_id').value or f"dock_{id(self) % 10000:04d}"
        self.coven_name = self.get_parameter('coven_name').value or get_coven_name()
        self.dock_x = self.get_parameter('dock_x').value
        self.dock_y = self.get_parameter('dock_y').value
        self.data_base_dir = os.path.expanduser(
            self.get_parameter('data_base_dir').value
        )

        # Session timestamp for data directory
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

        # Serial port state
        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._frame_parser = FrameParser()
        self._running = True

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Delegates
        self.topics = TopicManager(self)
        self.protocol = ProtocolHandler(self)
        self.data_proc = DataBatchProcessor(self)

        # COVEN protocol publisher (for dock integration)
        self.coven_pub = self.create_publisher(String, '/coven/bridge_events', 10)

        # Task auctioneer for mission dispatch
        self.auctioneer = TaskAuctioneer(
            dock_id=self.dock_id,
            dock_position=(self.dock_x, self.dock_y),
            bid_timeout=2.0,
        )
        self.auctioneer.set_send_task_callback(self._send_task_from_auctioneer)
        self.auctioneer.set_mission_complete_callback(self._on_mission_complete)

        # Mission dispatch subscriber
        self.mission_sub = self.create_subscription(
            String, '/coven/mission_request', self._mission_request_callback, 10
        )

        # Auctioneer status publisher
        self.auctioneer_status_pub = self.create_publisher(
            String, '/coven/auctioneer_status', 10
        )

        # Publisher for notifying offline SLAM processor
        self.sensor_batch_pub = self.create_publisher(
            String, '/coven/sensor_batch', 10
        )

        # Publisher for rover status (consumed by frontier_dispatcher)
        self.rover_status_pub = self.create_publisher(
            String, '/coven/rover_status', 10
        )

        # Timers
        self.create_timer(0.01, self.topics.process_publish_queues)  # 100 Hz
        self.create_timer(1.0, self._check_heartbeats)
        self.create_timer(2.0, self._try_dispatch_missions)
        self.create_timer(5.0, self._publish_auctioneer_status)

        # Start serial reader in background thread
        self._serial_thread = threading.Thread(
            target=self._serial_loop, daemon=True
        )
        self._serial_thread.start()

        self.get_logger().info(
            f"RoverBridge started — Dock: {self.dock_id}, "
            f"Coven: {self.coven_name}, Port: {self.serial_port_path}"
        )

    # -------------------------------------------------------------------------
    # Serial Communication
    # -------------------------------------------------------------------------

    def _serial_loop(self):
        """Background thread: open serial port, read frames, dispatch."""
        while self._running:
            try:
                self._open_serial()
                if self._serial and self._serial.is_open:
                    self._handle_serial_connection()
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

            # Clean up any connected rover on disconnect
            self._handle_serial_disconnect()

            if self._running:
                self.get_logger().info(
                    f"Retrying serial in {SERIAL_RETRY_DELAY}s..."
                )
                time.sleep(SERIAL_RETRY_DELAY)

    def _detect_serial_port(self) -> Optional[str]:
        """Scan for available serial ports (ACM for Pi Zero, USB for Arduino)."""
        candidates = sorted(
            glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        )
        if candidates:
            return candidates[0]
        return None

    def _open_serial(self):
        """Try to open the serial port (auto-detect if configured)."""
        port = self.serial_port_path
        if port == 'auto':
            port = self._detect_serial_port()
            if not port:
                self.get_logger().debug("No serial devices found")
                time.sleep(SERIAL_RETRY_DELAY)
                return

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=self.baud_rate,
                timeout=0.1,  # 100ms read timeout for non-blocking
            )
            self._frame_parser.reset()
            self.get_logger().info(
                f"Serial port opened: {port} @ {self.baud_rate}"
            )
        except serial.SerialException as e:
            self._serial = None
            self.get_logger().debug(
                f"Serial port not available: {port} ({e})"
            )
            time.sleep(SERIAL_RETRY_DELAY)

    def _handle_serial_connection(self):
        """Handle an open serial connection — read frames, dispatch."""
        # Create a pending rover for this connection
        rover = ConnectedRover(
            module_id="pending",
            state=RoverState.CONNECTING,
        )

        # Send IDENTIFY_REQUEST to start handshake
        self.send_frame(
            encode_identify_request(self.dock_id, self.coven_name)
        )

        while self._running and self._serial and self._serial.is_open:
            try:
                data = self._serial.read(256)
                if not data:
                    continue

                frames = self._frame_parser.feed(data)
                for msg_type, payload in frames:
                    decoded = decode_message(msg_type, payload)
                    if decoded:
                        self.protocol.process_message(rover, decoded)
                        # Update rover reference if identity changed
                        with self.rovers_lock:
                            if rover.module_id in self.rovers:
                                rover = self.rovers[rover.module_id]

            except serial.SerialException as e:
                self.get_logger().warning(f"Serial read error: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"Frame processing error: {e}")

    def _handle_serial_disconnect(self):
        """Clean up when serial connection drops."""
        with self._serial_lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

        # Disconnect all rovers (there should be at most one)
        with self.rovers_lock:
            rover_ids = list(self.rovers.keys())

        for module_id in rover_ids:
            with self.rovers_lock:
                rover = self.rovers.get(module_id)
            if rover:
                self._disconnect_rover(rover)

    def send_frame(self, frame: bytes):
        """Send a pre-built binary frame over serial."""
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(frame)
                    self._serial.flush()
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write error: {e}")

    # -------------------------------------------------------------------------
    # Connection Lifecycle
    # -------------------------------------------------------------------------

    def _disconnect_rover(self, rover: ConnectedRover):
        """Clean up a disconnected rover."""
        self.get_logger().info(f"Rover {rover.module_id} disconnected")

        # Calculate namespace hold timeout
        if rover.mission_estimated_path_m > 0 and rover.mission_max_speed_mps > 0:
            estimated_time = rover.mission_estimated_path_m / rover.mission_max_speed_mps
            namespace_timeout = estimated_time * MISSION_TIMEOUT_FACTOR
            self.get_logger().info(
                f"Holding namespace '{rover.module_id}' for {namespace_timeout:.0f}s "
                f"(500% of {estimated_time:.0f}s estimated mission time)"
            )
        else:
            namespace_timeout = DEFAULT_NAMESPACE_HOLD_TIME
            self.get_logger().info(
                f"Holding namespace '{rover.module_id}' for {namespace_timeout:.0f}s (default)"
            )

        mark_witch_missing(rover.module_id, namespace_timeout)

        with self.rovers_lock:
            if rover.module_id in self.rovers:
                del self.rovers[rover.module_id]

        self.topics.teardown_rover(rover.module_id)
        self._publish_event("ROVER_DISCONNECTED", rover.module_id, {
            "namespace_hold_timeout": namespace_timeout,
        })

    def _check_heartbeats(self):
        """Check for timed-out rovers and clean up disconnected ones."""
        now = time.time()
        timed_out = []

        with self.rovers_lock:
            for module_id, rover in self.rovers.items():
                if rover.state != RoverState.DISCONNECTED and \
                   now - rover.last_heartbeat > HEARTBEAT_TIMEOUT:
                    timed_out.append(module_id)

        for module_id in timed_out:
            self.get_logger().warning(f"Rover {module_id} heartbeat timeout")
            with self.rovers_lock:
                rover = self.rovers.get(module_id)
            if rover:
                self._disconnect_rover(rover)

    def get_connected_rovers(self) -> List[str]:
        """Get list of connected rover IDs."""
        with self.rovers_lock:
            return [
                mid for mid, r in self.rovers.items()
                if r.state == RoverState.ACTIVE
            ]

    # -------------------------------------------------------------------------
    # Publishing Helpers
    # -------------------------------------------------------------------------

    def _publish_rover_status(self, rover: ConnectedRover, mission_status: str):
        """Publish rover status for frontier_dispatcher."""
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
            "current_mission": rover.current_mission_id,
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

    # -------------------------------------------------------------------------
    # Auctioneer Integration
    # -------------------------------------------------------------------------

    def _send_task_from_auctioneer(self, module_id: str, task_req: dict):
        """Callback from auctioneer to send TASK_REQ to a rover."""
        with self.rovers_lock:
            rover = self.rovers.get(module_id)
            if not rover or rover.state != RoverState.ACTIVE:
                self.get_logger().error(f"Cannot send task to {module_id}: not active")
                return

        # Extract mission info for namespace lifecycle tracking
        task_id = task_req.get("task_id", "unknown")
        waypoints = task_req.get("waypoints", [])

        estimated_path = 0.0
        if waypoints:
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
            estimated_path += math.sqrt(prev_x**2 + prev_y**2)

        rover.current_mission_id = task_id
        rover.mission_estimated_path_m = estimated_path
        rover.mission_max_speed_mps = 0.3

        self.get_logger().info(
            f"Mission {task_id}: estimated path {estimated_path:.1f}m, "
            f"timeout will be {(estimated_path / rover.mission_max_speed_mps) * MISSION_TIMEOUT_FACTOR:.0f}s"
        )

        # Build and send task as binary frame
        task_req["dock_id"] = self.dock_id
        task_req["module_id"] = module_id
        self.send_frame(encode_task_request(task_req))
        self.get_logger().info(f"Sent TASK_REQ to {module_id}: {task_id}")

    def _on_mission_complete(self, mission: Mission, success: bool):
        """Callback from auctioneer when mission completes."""
        self.get_logger().info(
            f"Mission {mission.mission_id} complete: "
            f"{'SUCCESS' if success else 'FAILED'}"
        )

    def _try_dispatch_missions(self):
        """Timer callback to try dispatching missions from queue."""
        # Check for timed-out missions first
        timed_out = self.auctioneer.check_timeouts()
        for mission_id in timed_out:
            self.get_logger().warning(f"Mission {mission_id} timed out — rover freed")

        if not self.auctioneer.mission_queue:
            return

        idle_count = len(self.auctioneer.get_idle_rovers())
        if idle_count == 0:
            return

        self.auctioneer.try_dispatch()

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

        task_type_str = data.get("type", data.get("task_type", "explore")).lower()
        waypoints = data.get("waypoints", [])
        priority = data.get("priority", 0)

        if not waypoints:
            self.get_logger().error("Mission request missing waypoints")
            return

        wp_tuples = []
        for wp in waypoints:
            if isinstance(wp, dict):
                wp_tuples.append((wp["x"], wp["y"]))
            else:
                wp_tuples.append((wp[0], wp[1]))

        task_type_map = {
            "explore":       TaskType.EXPLORE,
            "spectral":      TaskType.SPECTRAL,
            "sample":        TaskType.SAMPLE,
            "deliver":       TaskType.DELIVER,
            "survey":        TaskType.SURVEY,
            "barometric":    TaskType.BAROMETRIC,
            "excavate":      TaskType.EXCAVATE,
            "haul":          TaskType.HAUL,
            "aerial_survey": TaskType.AERIAL_SURVEY,
        }
        task_type = task_type_map.get(task_type_str, TaskType.EXPLORE)

        import uuid
        mission = Mission(
            mission_id=f"{task_type_str}_{uuid.uuid4().hex[:8]}",
            task_type=task_type,
            waypoints=wp_tuples,
            dock_return=(0.0, 0.0),
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

        task_data["dock_id"] = self.dock_id
        task_data["task_id"] = task_id
        task_data["module_id"] = module_id
        self.send_frame(encode_task_request(task_data))
        return True

    def destroy_node(self):
        """Clean shutdown."""
        self._running = False
        with self._serial_lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
        super().destroy_node()


# Main
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
