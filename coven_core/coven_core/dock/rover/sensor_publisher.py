"""
sensor_publisher.py - Dock-Centric Sensor Publisher

Collects sensor data from the rover (LiDAR, odometry, battery) and
publishes bundled SensorData messages to the dock at regular intervals.

This is the rover-side component of the dock-centric architecture.
All heavy processing (SLAM, Nav2, exploration) happens on the dock.
The rover just streams sensor data and executes velocity commands.

Author: Alexander Shultis
Date: December 2025
"""

import logging
import math
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from coven_core.common import SensorData, sensor_data_encode

logger = logging.getLogger(__name__)


class SensorPublisher:
    """
    Collects and publishes bundled sensor data to the dock.

    In dock-centric mode, the rover publishes raw sensor readings
    and the dock handles SLAM, navigation, and exploration planning.

    Subscribes to:
        - /{namespace}/scan (LaserScan)
        - /{namespace}/odom (Odometry)

    Publishes to:
        - /coven/sensor_data (String - JSON encoded SensorData)

    Usage:
        publisher = SensorPublisher(node, "Hermione_Granger", publish_rate=10.0)
        # Sensor data will be published automatically via timer
    """

    def __init__(
        self,
        node: Node,
        module_id: str,
        publish_rate: float = 10.0,
        battery_callback: Optional[Callable[[], float]] = None
    ):
        """
        Initialize the sensor publisher.

        Args:
            node: ROS2 node to attach subscriptions/publishers to
            module_id: Unique identifier for this rover
            publish_rate: Rate to publish bundled data (Hz)
            battery_callback: Optional callback that returns current battery level (0.0-1.0)
        """
        self._node = node
        self._module_id = module_id
        self._battery_callback = battery_callback or (lambda: 1.0)

        # Cached sensor data
        self._latest_scan: Optional[LaserScan] = None
        self._latest_odom: Optional[Odometry] = None

        # QoS for sensor data - best effort for high frequency
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS for publishing to dock - reliable
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to local sensors
        namespace = module_id
        self._scan_sub = node.create_subscription(
            LaserScan,
            f'/{namespace}/scan',
            self._on_scan,
            sensor_qos
        )
        self._odom_sub = node.create_subscription(
            Odometry,
            f'/{namespace}/odom',
            self._on_odom,
            sensor_qos
        )

        # Publisher for bundled sensor data to dock
        self._sensor_pub = node.create_publisher(
            String,
            '/coven/sensor_data',
            reliable_qos
        )

        # Timer for publishing at fixed rate
        period = 1.0 / publish_rate
        self._timer = node.create_timer(period, self._publish_sensor_data)

        logger.info(f"[{module_id}] SensorPublisher initialized at {publish_rate} Hz")

    def _on_scan(self, msg: LaserScan) -> None:
        """Cache latest LiDAR scan."""
        self._latest_scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        """Cache latest odometry."""
        self._latest_odom = msg

    def _publish_sensor_data(self) -> None:
        """Bundle and publish sensor data to dock."""
        # Build SensorData message
        data = SensorData(module_id=self._module_id)

        # Timestamp
        now = self._node.get_clock().now()
        data.timestamp = now.nanoseconds / 1e9

        # LiDAR scan
        if self._latest_scan is not None:
            scan = self._latest_scan
            # Convert ranges to list, replacing inf with max_range
            ranges = []
            for r in scan.ranges:
                if math.isinf(r) or math.isnan(r):
                    ranges.append(scan.range_max)
                else:
                    ranges.append(float(r))
            data.scan_ranges = ranges
            data.scan_angle_min = float(scan.angle_min)
            data.scan_angle_max = float(scan.angle_max)
            data.scan_angle_increment = float(scan.angle_increment)

        # Odometry
        if self._latest_odom is not None:
            odom = self._latest_odom
            data.odom_x = float(odom.pose.pose.position.x)
            data.odom_y = float(odom.pose.pose.position.y)

            # Extract yaw from quaternion
            q = odom.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            data.odom_theta = float(math.atan2(siny_cosp, cosy_cosp))

            data.odom_vx = float(odom.twist.twist.linear.x)
            data.odom_vtheta = float(odom.twist.twist.angular.z)

        # Battery level
        data.battery_level = self._battery_callback()

        # Publish
        msg = String()
        msg.data = sensor_data_encode(data)
        self._sensor_pub.publish(msg)

    def has_scan(self) -> bool:
        """Check if we have received at least one scan."""
        return self._latest_scan is not None

    def has_odom(self) -> bool:
        """Check if we have received at least one odom."""
        return self._latest_odom is not None

    def is_ready(self) -> bool:
        """Check if we have all required sensor data."""
        return self.has_scan() and self.has_odom()

    def get_latest_pose(self) -> tuple:
        """
        Get the latest known pose (x, y, theta).

        Returns:
            Tuple of (x, y, theta) or (0, 0, 0) if no odom received
        """
        if self._latest_odom is None:
            return (0.0, 0.0, 0.0)

        odom = self._latest_odom
        x = float(odom.pose.pose.position.x)
        y = float(odom.pose.pose.position.y)

        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = float(math.atan2(siny_cosp, cosy_cosp))

        return (x, y, theta)

    def shutdown(self) -> None:
        """Clean up resources."""
        if self._timer is not None:
            self._timer.cancel()
        logger.info(f"[{self._module_id}] SensorPublisher shutdown")
