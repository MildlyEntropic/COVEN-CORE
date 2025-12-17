#!/usr/bin/env python3
"""
Odom to TF Broadcaster

Subscribes to /odom and broadcasts the odom->base_link transform.
This is needed because Gazebo's TF bridge doesn't properly convert
the Pose_V messages to TF2 transforms.

Author: Alexander Shultis
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    """Broadcasts odom->base_link transform from odometry messages."""

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Declare parameters
        self.declare_parameter('odom_topic', 'odom')  # Use relative topic for remapping
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'Broadcasting TF: {self.odom_frame} -> {self.base_frame} from {odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        """Convert odometry message to TF transform and broadcast."""
        t = TransformStamped()

        # Use the odometry timestamp
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Copy position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy orientation
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
