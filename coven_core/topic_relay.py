#!/usr/bin/env python3
"""
topic_relay.py - Simple Topic Relay Node

Relays messages from one topic to another.
Used to bridge rover-specific topics to generic Nav2 topics.

Author: Alexander Shultis
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TopicRelay(Node):
    """Relays messages between topics."""

    def __init__(self):
        super().__init__('topic_relay')

        # Declare parameters
        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('msg_type', 'twist')  # twist, odom, scan

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        msg_type = self.get_parameter('msg_type').value

        if not input_topic or not output_topic:
            self.get_logger().error('input_topic and output_topic parameters required')
            return

        # Set up QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create appropriate sub/pub based on message type
        if msg_type == 'twist':
            self.sub = self.create_subscription(Twist, input_topic, self._relay_twist, qos)
            self.pub = self.create_publisher(Twist, output_topic, qos)
        elif msg_type == 'odom':
            self.sub = self.create_subscription(Odometry, input_topic, self._relay_odom, qos)
            self.pub = self.create_publisher(Odometry, output_topic, qos)
        elif msg_type == 'scan':
            # LaserScan typically uses best effort for performance
            scan_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
            self.sub = self.create_subscription(LaserScan, input_topic, self._relay_scan, scan_qos)
            self.pub = self.create_publisher(LaserScan, output_topic, scan_qos)
        else:
            self.get_logger().error(f'Unknown msg_type: {msg_type}')
            return

        self.get_logger().info(f'Relaying {msg_type}: {input_topic} -> {output_topic}')

    def _relay_twist(self, msg: Twist):
        self.pub.publish(msg)

    def _relay_odom(self, msg: Odometry):
        self.pub.publish(msg)

    def _relay_scan(self, msg: LaserScan):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
