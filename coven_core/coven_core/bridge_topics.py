"""
bridge_topics.py — Per-rover ROS2 topic management.

Creates and destroys per-rover publishers/subscribers, manages
thread-safe publish queues, and broadcasts TF transforms.
Instantiated by the RoverBridge node.

Author: Alexander Shultis
Date: January 2025
"""

from __future__ import annotations

import logging
import math
import queue
from typing import Dict, TYPE_CHECKING

logger = logging.getLogger(__name__)

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

if TYPE_CHECKING:
    from coven_core.rover_bridge import RoverBridge, ConnectedRover


class TopicManager:
    """Manages per-rover ROS2 publishers, subscribers, and TF broadcasts."""

    def __init__(self, bridge: 'RoverBridge'):
        self._bridge = bridge

        # Per-rover publishers (created dynamically)
        self.scan_pubs: Dict[str, any] = {}
        self.odom_pubs: Dict[str, any] = {}
        self.cmd_vel_subs: Dict[str, any] = {}

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(bridge)

        # Message queues for thread-safe publishing
        self.odom_queue: queue.Queue = queue.Queue()
        self.scan_queue: queue.Queue = queue.Queue()
        self.tf_queue: queue.Queue = queue.Queue()

    def setup_rover(self, module_id: str):
        """Create ROS2 publishers and subscribers for a rover."""
        self.scan_pubs[module_id] = self._bridge.create_publisher(
            LaserScan,
            f'/{module_id}/scan',
            self._bridge.sensor_qos
        )

        self.odom_pubs[module_id] = self._bridge.create_publisher(
            Odometry,
            f'/{module_id}/odom',
            self._bridge.sensor_qos
        )

        self.cmd_vel_subs[module_id] = self._bridge.create_subscription(
            Twist,
            f'/{module_id}/cmd_vel',
            lambda msg, mid=module_id: self._cmd_vel_callback(mid, msg),
            10
        )

        self._bridge.get_logger().info(f"Created topics for rover {module_id}")

    def teardown_rover(self, module_id: str):
        """Remove ROS2 publishers and subscribers for a rover."""
        if module_id in self.scan_pubs:
            self._bridge.destroy_publisher(self.scan_pubs.pop(module_id))
        if module_id in self.odom_pubs:
            self._bridge.destroy_publisher(self.odom_pubs.pop(module_id))
        if module_id in self.cmd_vel_subs:
            self._bridge.destroy_subscription(self.cmd_vel_subs.pop(module_id))

    def process_publish_queues(self):
        """Process message queues — called from ROS2 timer (main thread)."""
        while not self.odom_queue.empty():
            try:
                module_id, odom = self.odom_queue.get_nowait()
                if module_id in self.odom_pubs:
                    self.odom_pubs[module_id].publish(odom)
            except Exception as e:
                logger.warning("Odom publish failed: %s", e)
                break

        while not self.scan_queue.empty():
            try:
                module_id, scan = self.scan_queue.get_nowait()
                if module_id in self.scan_pubs:
                    self.scan_pubs[module_id].publish(scan)
            except Exception as e:
                logger.warning("Scan publish failed: %s", e)
                break

        while not self.tf_queue.empty():
            try:
                transform = self.tf_queue.get_nowait()
                self.tf_broadcaster.sendTransform(transform)
            except Exception as e:
                logger.warning("TF publish failed: %s", e)
                break

    def publish_odom_threadsafe(self, module_id: str, odom: Odometry):
        """Queue odom for thread-safe publishing."""
        self.odom_queue.put((module_id, odom))

    def publish_scan_threadsafe(self, module_id: str, scan: LaserScan):
        """Queue scan for thread-safe publishing."""
        self.scan_queue.put((module_id, scan))

    def publish_tf_threadsafe(self, transform: TransformStamped):
        """Queue TF for thread-safe publishing."""
        self.tf_queue.put(transform)

    def publish_rover_tf(self, rover: 'ConnectedRover'):
        """Publish TF transform for a rover."""
        t = TransformStamped()
        t.header.stamp = self._bridge.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = f"{rover.module_id}/base_link"

        t.transform.translation.x = rover.x
        t.transform.translation.y = rover.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(rover.theta / 2.0)
        t.transform.rotation.w = math.cos(rover.theta / 2.0)

        self.publish_tf_threadsafe(t)

    def _cmd_vel_callback(self, module_id: str, msg: Twist):
        """Handle cmd_vel from ROS2, forward to rover as binary frame."""
        from coven_core.frame_codec import encode_cmd_vel

        with self._bridge.rovers_lock:
            rover = self._bridge.rovers.get(module_id)
            if not rover:
                return
            from coven_core.rover_bridge import RoverState
            if rover.state != RoverState.ACTIVE:
                return

        self._bridge.send_frame(
            encode_cmd_vel(msg.linear.x, msg.angular.z)
        )
