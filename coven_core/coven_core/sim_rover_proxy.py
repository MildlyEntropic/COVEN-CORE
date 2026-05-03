#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
sim_rover_proxy.py — Gazebo-to-COVEN bridge node.

Each instance of this node represents one simulated rover. It subscribes to
the rover's Gazebo-bridged ROS2 topics (/<rover>/scan, /<rover>/odometry)
and re-emits that sensor stream over a virtual UART (PTY pair) using the
exact COVEN wire format the Rust firmware produces. The dock's
rover_bridge node — running unmodified — opens the slave end of the PTY,
sees a normal COVEN rover, and dispatches to it through the standard
auctioneer / capability-bitmask machinery.

This is the simulation equivalent of physically plugging a Pi Zero into the
dock's USB. The dock has no way to distinguish a sim-proxied rover from a
real one at the protocol layer.

Architecture:
    Gazebo simulated robot
        /<rover>/scan, /<rover>/odometry, /<rover>/cmd_vel
                  ↓ (subscribed by)
    sim_rover_proxy node (this file)
        ├── ROS2 subscribers + publisher
        ├── Owns a PTY pair via os.openpty()
        ├── Walks the COVEN FSM: BOOT → IDENTIFY → WAIT_VERIFY → NORMAL
        ├── Streams ScanData/OdomData (DATA_FRAME subtype 0x20 JSON)
        ├── Heartbeats at 1 Hz
        └── On TASK_REQ: publishes /<rover>/cmd_vel toward waypoints
                  ↓ (slave end of PTY at /dev/pts/N)
    dock rover_bridge node (unmodified)

Usage (from a launch file or shell):
    ros2 run coven_core sim_rover_proxy \
        --ros-args \
        -p rover_name:=witch_morgan \
        -p coven_name:=The_Graeae \
        -p capabilities:=0x03 \
        -p module_type:=ReconRover

The node logs the slave-side PTY device path on startup; the dock's
rover_bridge can be pointed at that path (or auto-detect via
/dev/ttyACM*/ttyUSB* if the proxy is given a stable symlink).

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import math
import os
import select
import struct
import threading
import time
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    HAS_ROS2 = True
except ImportError:  # pragma: no cover - allows static-analysis without ROS2
    HAS_ROS2 = False
    rclpy = None  # type: ignore
    Node = object  # type: ignore

from coven_core.frame_codec import FrameParser
from coven_core.rover_codec import (
    encode_heartbeat,
    encode_identify_reply,
    encode_odom_data_json,
    encode_scan_data_json,
    encode_task_ack_json,
    encode_task_complete_json,
    encode_task_start_json,
    encode_verify_rep,
)


# COVEN message types (dock → rover)
MSG_IDENTIFY_REQUEST = 0x01
MSG_VERIFY_OK = 0x03
MSG_VERIFY_FAIL = 0x04
MSG_IDENTIFY_ACK = 0x05
MSG_DATA_FRAME = 0x10
MSG_SYSTEM_PING = 0xFF


class SimRoverProxy(Node):
    """One Gazebo rover proxied as a COVEN protocol speaker on a PTY."""

    def __init__(self) -> None:
        super().__init__('sim_rover_proxy')

        # Parameters
        self.declare_parameter('rover_name', 'witch_morgan')
        self.declare_parameter('coven_name', 'The_Graeae')
        self.declare_parameter('module_type', 'ReconRover')
        self.declare_parameter('firmware', '0.1.0')
        # Capabilities accepted as a string so launch files can pass "0x03".
        self.declare_parameter('capabilities', '0x03')
        self.declare_parameter('battery_pct', 95.0)
        self.declare_parameter('heartbeat_hz', 1.0)
        self.declare_parameter('scan_relay_hz', 5.0)
        self.declare_parameter('odom_relay_hz', 10.0)
        # Per-rover Gazebo topic prefix. Defaults to the rover name (which is
        # also the Gazebo model name in the Dec 2025 launch files).
        self.declare_parameter('gz_topic_ns', '')
        # Optional symlink target so the dock can find the proxy on a stable
        # path (e.g. /dev/ttyVwitch_morgan). Empty disables.
        self.declare_parameter('pty_symlink', '')

        self.rover_name = self.get_parameter('rover_name').value
        self.coven_name = self.get_parameter('coven_name').value
        self.module_type = self.get_parameter('module_type').value
        self.firmware = self.get_parameter('firmware').value
        caps_str = self.get_parameter('capabilities').value
        self.capabilities = int(caps_str, 0) if isinstance(caps_str, str) else int(caps_str)
        self.battery_pct = float(self.get_parameter('battery_pct').value)
        self.heartbeat_period = 1.0 / max(0.1, float(self.get_parameter('heartbeat_hz').value))
        self.scan_relay_period = 1.0 / max(0.1, float(self.get_parameter('scan_relay_hz').value))
        self.odom_relay_period = 1.0 / max(0.1, float(self.get_parameter('odom_relay_hz').value))
        gz_ns = self.get_parameter('gz_topic_ns').value
        self.gz_topic_ns = gz_ns if gz_ns else self.rover_name
        self.pty_symlink = self.get_parameter('pty_symlink').value

        # COVEN protocol state
        self.state = 'BOOT'
        self.dock_id: Optional[str] = None
        self.assigned_name: Optional[str] = None
        self.parser = FrameParser()

        # Latest sensor data, populated by ROS2 subscribers
        self._lock = threading.Lock()
        self._latest_scan: Optional[LaserScan] = None
        self._latest_odom: Optional[Odometry] = None
        self._cmd_vel_target: Optional[Tuple[float, float, float]] = None  # (x,y,task_id)
        self._current_task_id: Optional[str] = None
        self._task_start_time: Optional[float] = None

        # Pose (for heartbeats)
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        # Open the PTY pair the dock will connect to.
        self._pty_master, self._pty_slave = os.openpty()
        self._slave_path = os.ttyname(self._pty_slave)
        self.get_logger().info(
            f"PTY slave path for dock to connect: {self._slave_path}"
        )
        if self.pty_symlink:
            try:
                if os.path.lexists(self.pty_symlink):
                    os.remove(self.pty_symlink)
                os.symlink(self._slave_path, self.pty_symlink)
                self.get_logger().info(f"PTY symlinked at {self.pty_symlink}")
            except OSError as e:
                self.get_logger().warn(
                    f"Could not create symlink {self.pty_symlink}: {e}"
                )

        # ROS2 subscriptions to Gazebo-bridged sensor topics.
        # Sensors typically use BEST_EFFORT in sim; commands use RELIABLE.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, f'/{self.gz_topic_ns}/scan', self._on_scan, sensor_qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry, f'/{self.gz_topic_ns}/odometry', self._on_odom, sensor_qos,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.gz_topic_ns}/cmd_vel', 10,
        )

        # Periodic timers
        self.create_timer(self.heartbeat_period, self._tick_heartbeat)
        self.create_timer(self.scan_relay_period, self._tick_scan_relay)
        self.create_timer(self.odom_relay_period, self._tick_odom_relay)
        self.create_timer(0.05, self._tick_navigation)
        # Drain incoming PTY bytes on a background thread.
        self._stop_evt = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f"sim_rover_proxy '{self.rover_name}' "
            f"(type={self.module_type}, caps=0x{self.capabilities:02x}) "
            f"initialized; awaiting dock IDENTIFY_REQUEST"
        )

    # -----------------------------------------------------------------
    # ROS2 subscriber callbacks
    # -----------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg
            # Cache pose for heartbeats.
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            # Convert quaternion to yaw.
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._theta = math.atan2(siny_cosp, cosy_cosp)

    # -----------------------------------------------------------------
    # PTY I/O
    # -----------------------------------------------------------------

    def _send(self, frame: bytes) -> None:
        """Write a complete COVEN frame to the PTY master."""
        try:
            os.write(self._pty_master, frame)
        except OSError as e:
            self.get_logger().warn(f"PTY write failed: {e}")

    def _reader_loop(self) -> None:
        """Background thread reading bytes from the dock and feeding the parser."""
        while not self._stop_evt.is_set():
            r, _, _ = select.select([self._pty_master], [], [], 0.1)
            if not r:
                continue
            try:
                data = os.read(self._pty_master, 4096)
            except OSError:
                continue
            if not data:
                continue
            for msg_type, payload in self.parser.feed(data):
                self._handle_dock_message(msg_type, payload)

    # -----------------------------------------------------------------
    # COVEN FSM — incoming dock messages
    # -----------------------------------------------------------------

    def _handle_dock_message(self, msg_type: int, payload: bytes) -> None:
        if msg_type == MSG_IDENTIFY_REQUEST:
            self._on_identify_request(payload)
        elif msg_type == MSG_IDENTIFY_ACK:
            self._on_identify_ack(payload)
        elif msg_type == MSG_VERIFY_OK:
            self._on_verify_ok()
        elif msg_type == MSG_VERIFY_FAIL:
            self._on_verify_fail(payload)
        elif msg_type == MSG_DATA_FRAME:
            self._on_data_frame(payload)
        elif msg_type == MSG_SYSTEM_PING:
            # No-op echo for liveness; dock-side handles pings itself.
            pass
        else:
            self.get_logger().debug(f"Ignoring unknown msg_type 0x{msg_type:02x}")

    def _on_identify_request(self, payload: bytes) -> None:
        # Layout: [dock_id_len][dock_id][coven_name_len][coven_name][assigned_name_len][assigned_name]
        try:
            i = 0
            dock_id_len = payload[i]; i += 1
            self.dock_id = payload[i:i + dock_id_len].decode('utf-8', errors='replace')
            i += dock_id_len
            coven_len = payload[i]; i += 1
            _coven_name = payload[i:i + coven_len].decode('utf-8', errors='replace')
            i += coven_len
            assigned_len = payload[i]; i += 1
            assigned = payload[i:i + assigned_len].decode('utf-8', errors='replace')
        except (IndexError, UnicodeDecodeError):
            self.get_logger().warn("Malformed IDENTIFY_REQUEST")
            return

        # Dock is asking us to claim our identity.
        self.get_logger().info(
            f"IDENTIFY_REQUEST from dock={self.dock_id} (assigned='{assigned}')"
        )
        reply = encode_identify_reply(
            module_id=self.rover_name,
            module_type=self.module_type,
            firmware=self.firmware,
            battery_pct=self.battery_pct,
            status="OK",
            capabilities=self.capabilities,
        )
        self._send(reply)
        self.state = 'IDENTIFY'

    def _on_identify_ack(self, payload: bytes) -> None:
        # Dock confirms our identity. Move to WAIT_VERIFY; no proactive
        # message — the dock will send VERIFY_REQ next.
        self.get_logger().info(f"IDENTIFY_ACK received; state -> WAIT_VERIFY")
        self.state = 'WAIT_VERIFY'

    def _on_verify_ok(self) -> None:
        self.get_logger().info("VERIFY_OK; rover is NORMAL and idle")
        self.state = 'NORMAL'
        # Send a VERIFY_REP confirming our self-checks (mock: always success).
        self._send(encode_verify_rep(
            module_id=self.rover_name,
            success=True,
            failed_checks=[],
            note="All systems nominal (sim)",
        ))

    def _on_verify_fail(self, payload: bytes) -> None:
        self.get_logger().warn("VERIFY_FAIL; rover is REJECTED")
        self.state = 'REJECTED'

    def _on_data_frame(self, payload: bytes) -> None:
        # Subtype byte is first.
        if not payload:
            return
        subtype = payload[0]
        body = payload[1:]
        if subtype == 0x10:  # SUBTYPE_TASK_MESSAGE
            self._on_task_message(body)
        elif subtype == 0x30:  # SUBTYPE_CMD_VEL
            if len(body) >= 8:
                linear, angular = struct.unpack('<ff', body[:8])
                self._publish_cmd_vel(linear, angular)
        # Other subtypes (verification, sensor, power) are not expected
        # from dock to rover in this proxy's role.

    def _on_task_message(self, body: bytes) -> None:
        try:
            import json
            obj = json.loads(body.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.get_logger().warn("Malformed TASK message")
            return

        if "TaskReq" in obj or "task_id" in obj:
            task = obj.get("TaskReq", obj)
            task_id = task.get("task_id", "unknown")
            waypoints = task.get("waypoints", [])
            if not waypoints:
                self.get_logger().warn(f"TASK_REQ {task_id} has no waypoints")
                return
            wp = waypoints[0]
            self._cmd_vel_target = (wp.get("x", 0.0), wp.get("y", 0.0), task_id)
            self._current_task_id = task_id
            self._task_start_time = time.time()
            self.state = 'FIELD_OPS'
            self.get_logger().info(
                f"TASK_REQ {task_id} → ({wp.get('x'):.2f}, {wp.get('y'):.2f}); FIELD_OPS"
            )
            # Acknowledge acceptance.
            self._send(encode_task_ack_json(
                module_id=self.rover_name, task_id=task_id, success=True,
            ))
            self._send(encode_task_start_json(
                module_id=self.rover_name, task_id=task_id, timestamp=time.time(),
            ))

    # -----------------------------------------------------------------
    # Periodic ticks
    # -----------------------------------------------------------------

    def _tick_heartbeat(self) -> None:
        if self.state in ('BOOT', 'REJECTED'):
            return
        status = {
            'NORMAL': 'IDLE',
            'FIELD_OPS': 'ACTIVE',
        }.get(self.state, 'STARTUP')
        with self._lock:
            x, y, theta = self._x, self._y, self._theta
        frame = encode_heartbeat(
            module_id=self.rover_name,
            battery_pct=self.battery_pct,
            mission_status=status,
            x=x, y=y, theta=theta,
        )
        self._send(frame)

    def _tick_scan_relay(self) -> None:
        if self.state not in ('NORMAL', 'FIELD_OPS'):
            return
        with self._lock:
            scan = self._latest_scan
        if scan is None:
            return
        # Convert LaserScan ranges (meters, float) to mm u16 for the wire format.
        ranges_mm = [
            min(int(r * 1000.0), 0xFFFF) if math.isfinite(r) else 0xFFFF
            for r in scan.ranges
        ]
        frame = encode_scan_data_json(
            module_id=self.rover_name,
            timestamp=scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
            angle_min=scan.angle_min,
            angle_max=scan.angle_max,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            ranges_mm=ranges_mm,
        )
        self._send(frame)

    def _tick_odom_relay(self) -> None:
        if self.state not in ('NORMAL', 'FIELD_OPS'):
            return
        with self._lock:
            odom = self._latest_odom
            x, y, theta = self._x, self._y, self._theta
        if odom is None:
            return
        frame = encode_odom_data_json(
            module_id=self.rover_name,
            timestamp=odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9,
            x=x, y=y, theta=theta,
            v_linear=odom.twist.twist.linear.x,
            v_angular=odom.twist.twist.angular.z,
        )
        self._send(frame)

    def _tick_navigation(self) -> None:
        """Drive the simulated rover toward its current waypoint via simple
        proportional control. This is not a real navigation stack; it's a
        minimum-viable mover so the FSM can exit FIELD_OPS by reaching the
        goal. Replace with Nav2 when the launch file is wired up."""
        if self.state != 'FIELD_OPS' or self._cmd_vel_target is None:
            return
        target_x, target_y, task_id = self._cmd_vel_target
        with self._lock:
            x, y, theta = self._x, self._y, self._theta
        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        # Goal-reached threshold.
        if dist < 0.3:
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().info(f"Task {task_id} reached goal; FIELD_OPS → NORMAL")
            duration = time.time() - (self._task_start_time or time.time())
            self._send(encode_task_complete_json(
                module_id=self.rover_name,
                task_id=task_id,
                success=True,
                coverage=1.0,
                duration=duration,
            ))
            self.state = 'NORMAL'
            self._cmd_vel_target = None
            self._current_task_id = None
            return
        # Proportional steering.
        target_heading = math.atan2(dy, dx)
        heading_err = target_heading - theta
        # Normalize.
        while heading_err > math.pi:
            heading_err -= 2 * math.pi
        while heading_err < -math.pi:
            heading_err += 2 * math.pi
        angular = max(-1.0, min(1.0, 1.5 * heading_err))
        # Slow down when off-heading.
        linear = max(0.0, min(0.4, 0.5 * dist)) * max(0.0, math.cos(heading_err))
        self._publish_cmd_vel(linear, angular)

    def _publish_cmd_vel(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    # -----------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------

    def destroy_node(self):
        self._stop_evt.set()
        try:
            os.close(self._pty_master)
        except OSError:
            pass
        try:
            os.close(self._pty_slave)
        except OSError:
            pass
        if self.pty_symlink and os.path.lexists(self.pty_symlink):
            try:
                os.remove(self.pty_symlink)
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    if not HAS_ROS2:
        raise RuntimeError(
            "ROS2 (rclpy) is not installed. This node must run inside a "
            "ROS2 environment — typically the COVEN sim Docker container."
        )
    rclpy.init(args=args)
    node = SimRoverProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
