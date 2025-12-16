#!/usr/bin/env python3
"""
Scan Frame Republisher

Subscribes to a LaserScan topic and republishes it with a modified frame_id.
This is needed for multi-robot setups where Gazebo publishes scans with
a generic 'base_link' frame but RViz2 needs namespaced frames like
'{robot}/base_link'.

Author: Alexander Shultis
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameRepublisher(Node):
    """Republishes LaserScan messages with a modified frame_id."""

    def __init__(self):
        super().__init__('scan_frame_republisher')

        # Declare parameters
        self.declare_parameter('input_topic', 'scan_in')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        # Subscriber
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info(
            f'Republishing {input_topic} -> {output_topic} with frame_id={self.frame_id}'
        )

    def scan_callback(self, msg: LaserScan):
        """Republish scan with modified frame_id."""
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
