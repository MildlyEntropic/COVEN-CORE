"""
sensor_receiver.py - Dock-Centric Sensor Receiver

Receives bundled sensor data from rovers and republishes to standard
ROS2 topics for SLAM and Nav2 consumption.

In dock-centric mode, rovers publish SensorData (bundled JSON), and
the dock unpacks this into standard LaserScan, Odometry, and TF messages.

Author: Alexander Shultis
Date: December 2025
"""

import logging
import math
from typing import Dict, Optional, Callable

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from coven_core.common import SensorData, sensor_data_decode

logger = logging.getLogger(__name__)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SensorReceiver:
    """
    Receives sensor data from rovers and republishes for SLAM/Nav2.

    In dock-centric architecture:
    - Rovers publish bundled SensorData to /coven/sensor_data
    - Dock unpacks and republishes to per-rover topics:
        - /{module_id}/scan (LaserScan)
        - /{module_id}/odom (Odometry)
        - TF: odom → base_link

    This allows SLAM and Nav2 to consume standard message types.

    Subscribes to:
        - /coven/sensor_data (String - JSON encoded SensorData)

    Publishes to (per rover):
        - /{module_id}/scan_dock (LaserScan)
        - /{module_id}/odom_dock (Odometry)
        - TF: {module_id}/odom → {module_id}/base_link
    """

    def __init__(
        self,
        node: Node,
        on_sensor_data: Optional[Callable[[SensorData], None]] = None
    ):
        """
        Initialize the sensor receiver.

        Args:
            node: ROS2 node to attach subscriptions/publishers to
            on_sensor_data: Optional callback when sensor data received
        """
        self._node = node
        self._on_sensor_data = on_sensor_data

        # Per-rover publishers (created on demand)
        self._scan_pubs: Dict[str, any] = {}
        self._odom_pubs: Dict[str, any] = {}

        # TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(node)

        # Cached latest sensor data per rover
        self._latest_data: Dict[str, SensorData] = {}

        # QoS for reliable communication
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for sensor republishing (best effort for high rate)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to bundled sensor data
        self._sensor_sub = node.create_subscription(
            String,
            '/coven/sensor_data',
            self._on_sensor_msg,
            reliable_qos
        )

        self._sensor_qos = sensor_qos
        logger.info("SensorReceiver initialized")

    def _get_or_create_publishers(self, module_id: str):
        """Get or create publishers for a rover."""
        if module_id not in self._scan_pubs:
            # Create scan publisher
            self._scan_pubs[module_id] = self._node.create_publisher(
                LaserScan,
                f'/{module_id}/scan_dock',
                self._sensor_qos
            )
            logger.info(f"Created scan publisher for '{module_id}'")

        if module_id not in self._odom_pubs:
            # Create odom publisher
            self._odom_pubs[module_id] = self._node.create_publisher(
                Odometry,
                f'/{module_id}/odom_dock',
                self._sensor_qos
            )
            logger.info(f"Created odom publisher for '{module_id}'")

    def _on_sensor_msg(self, msg: String) -> None:
        """Handle incoming bundled sensor data."""
        data = sensor_data_decode(msg)
        if data is None:
            return

        module_id = data.module_id

        # Cache latest data
        self._latest_data[module_id] = data

        # Ensure publishers exist
        self._get_or_create_publishers(module_id)

        # Get current ROS time
        now = self._node.get_clock().now()

        # Republish scan
        if data.scan_ranges:
            self._publish_scan(module_id, data, now)

        # Republish odometry and TF
        self._publish_odom(module_id, data, now)
        self._publish_tf(module_id, data, now)

        # Callback
        if self._on_sensor_data:
            self._on_sensor_data(data)

    def _publish_scan(self, module_id: str, data: SensorData, stamp) -> None:
        """Republish LaserScan from bundled data."""
        scan = LaserScan()
        scan.header.stamp = stamp.to_msg()
        scan.header.frame_id = f'{module_id}/base_scan'

        scan.angle_min = data.scan_angle_min
        scan.angle_max = data.scan_angle_max
        scan.angle_increment = data.scan_angle_increment

        # Calculate range values
        num_readings = len(data.scan_ranges)
        if num_readings > 0 and data.scan_angle_increment > 0:
            scan.time_increment = 0.0  # Instantaneous scan
            scan.scan_time = 0.1  # 10 Hz assumption
            scan.range_min = 0.1
            scan.range_max = 12.0
            scan.ranges = data.scan_ranges
            scan.intensities = []  # No intensity data

            self._scan_pubs[module_id].publish(scan)

    def _publish_odom(self, module_id: str, data: SensorData, stamp) -> None:
        """Republish Odometry from bundled data."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = f'{module_id}/odom'
        odom.child_frame_id = f'{module_id}/base_link'

        # Position
        odom.pose.pose.position.x = data.odom_x
        odom.pose.pose.position.y = data.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(data.odom_theta)

        # Velocity
        odom.twist.twist.linear.x = data.odom_vx
        odom.twist.twist.angular.z = data.odom_vtheta

        self._odom_pubs[module_id].publish(odom)

    def _publish_tf(self, module_id: str, data: SensorData, stamp) -> None:
        """Publish odom → base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = f'{module_id}/odom'
        t.child_frame_id = f'{module_id}/base_link'

        t.transform.translation.x = data.odom_x
        t.transform.translation.y = data.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(data.odom_theta)

        self._tf_broadcaster.sendTransform(t)

    def get_latest_data(self, module_id: str) -> Optional[SensorData]:
        """Get latest sensor data for a rover."""
        return self._latest_data.get(module_id)

    def get_rover_position(self, module_id: str) -> Optional[tuple]:
        """
        Get rover position from latest sensor data.

        Returns:
            Tuple of (x, y, theta) or None if no data
        """
        data = self._latest_data.get(module_id)
        if data is None:
            return None
        return (data.odom_x, data.odom_y, data.odom_theta)

    def get_all_positions(self) -> Dict[str, tuple]:
        """Get positions of all rovers with sensor data."""
        return {
            module_id: (data.odom_x, data.odom_y, data.odom_theta)
            for module_id, data in self._latest_data.items()
        }

    def shutdown(self) -> None:
        """Clean up resources."""
        logger.info("SensorReceiver shutdown")
