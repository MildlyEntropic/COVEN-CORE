#!/usr/bin/env python3
"""
encoder_odom.py - Wheel Encoder Odometry Publisher

Reads quadrature encoders from differential drive robot and publishes
nav_msgs/Odometry messages plus odom->base_link TF transform.

Hardware: Quadrature encoders on N20 gear motors
- Uses pigpio for interrupt-based encoder reading
- Calculates odometry from wheel tick counts

Author: Alexander Shultis
Date: January 2026
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import math
import time
import threading

# pigpio required for interrupt-based encoder reading
try:
    import pigpio
    USE_PIGPIO = True
except ImportError:
    USE_PIGPIO = False


class EncoderOdometry(Node):
    """
    Publishes odometry computed from wheel encoders.

    Publishes: /{namespace}/odom (nav_msgs/Odometry)
    Broadcasts: odom -> base_link transform (optional)

    Uses quadrature decoding for direction detection.
    """

    # Default GPIO pins for encoders (BCM numbering)
    DEFAULT_PINS = {
        'left_a': 23,
        'left_b': 24,
        'right_a': 27,
        'right_b': 22,
    }

    def __init__(self):
        super().__init__('encoder_odom')

        # Declare parameters
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('wheel_base', 0.1)         # meters between wheels
        self.declare_parameter('wheel_radius', 0.03)      # wheel radius in meters
        self.declare_parameter('encoder_ppr', 210)        # pulses per revolution (after gearbox)
        self.declare_parameter('publish_rate', 50.0)      # Hz
        self.declare_parameter('publish_tf', True)        # Publish odom->base_link TF
        self.declare_parameter('invert_left', False)      # Invert left encoder direction
        self.declare_parameter('invert_right', False)     # Invert right encoder direction

        # GPIO pin parameters
        self.declare_parameter('pin_left_a', self.DEFAULT_PINS['left_a'])
        self.declare_parameter('pin_left_b', self.DEFAULT_PINS['left_b'])
        self.declare_parameter('pin_right_a', self.DEFAULT_PINS['right_a'])
        self.declare_parameter('pin_right_b', self.DEFAULT_PINS['right_b'])

        # Get parameter values
        self.namespace = self.get_parameter('robot_namespace').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_ppr = self.get_parameter('encoder_ppr').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value

        # Get pin assignments
        self.pins = {
            'left_a': self.get_parameter('pin_left_a').value,
            'left_b': self.get_parameter('pin_left_b').value,
            'right_a': self.get_parameter('pin_right_a').value,
            'right_b': self.get_parameter('pin_right_b').value,
        }

        # Distance per encoder tick
        self.meters_per_tick = (2 * math.pi * self.wheel_radius) / self.encoder_ppr

        # Odometry state (robot pose in odom frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0

        # Encoder tick counts (updated by interrupt handlers)
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left = 0
        self.last_right = 0
        self.last_time = time.time()

        # Lock for thread-safe encoder access
        self.lock = threading.Lock()

        # Track initialization state
        self.gpio_initialized = False
        self.pi = None
        self.callbacks = []

        # Initialize encoder reading
        self._init_encoders()

        # Publisher
        topic = f'/{self.namespace}/odom' if self.namespace else '/odom'
        self.odom_pub = self.create_publisher(Odometry, topic, 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_odom)

        self.get_logger().info(f'Encoder odometry initialized, publishing to {topic}')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m, PPR: {self.encoder_ppr}')
        self.get_logger().info(f'Meters per tick: {self.meters_per_tick:.6f}')

    def _init_encoders(self):
        """Set up encoder GPIO pins and interrupt callbacks."""
        if not USE_PIGPIO:
            self.get_logger().error('pigpio required for encoder reading')
            self.get_logger().error('Install: sudo apt install pigpio python3-pigpio')
            self.get_logger().error('Start daemon: sudo pigpiod')
            return

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Cannot connect to pigpio daemon')
            self.get_logger().error('Start it with: sudo pigpiod')
            return

        # Set up pins as inputs with pull-ups
        for pin_name, pin in self.pins.items():
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)

        # Set up callbacks for quadrature decoding
        # We only need to watch channel A - channel B tells us direction
        cb_left = self.pi.callback(
            self.pins['left_a'], pigpio.EITHER_EDGE, self._left_callback
        )
        cb_right = self.pi.callback(
            self.pins['right_a'], pigpio.EITHER_EDGE, self._right_callback
        )
        self.callbacks = [cb_left, cb_right]

        self.gpio_initialized = True
        self.get_logger().info('Encoder interrupts configured via pigpio')

    def _left_callback(self, gpio, level, tick):
        """
        Left encoder interrupt handler.

        Quadrature decoding: if A and B are same, moving one direction;
        if different, moving the other direction.
        """
        if not self.pi:
            return

        a = self.pi.read(self.pins['left_a'])
        b = self.pi.read(self.pins['left_b'])

        with self.lock:
            if a == b:
                self.left_ticks += 1
            else:
                self.left_ticks -= 1

            # Apply inversion if configured
            if self.invert_left:
                self.left_ticks = -self.left_ticks

    def _right_callback(self, gpio, level, tick):
        """
        Right encoder interrupt handler.

        Note: Right side typically needs opposite counting direction
        due to motor mounting orientation.
        """
        if not self.pi:
            return

        a = self.pi.read(self.pins['right_a'])
        b = self.pi.read(self.pins['right_b'])

        with self.lock:
            # Right side reversed (motors face opposite directions)
            if a == b:
                self.right_ticks -= 1
            else:
                self.right_ticks += 1

            # Apply inversion if configured
            if self.invert_right:
                self.right_ticks = -self.right_ticks

    def _publish_odom(self):
        """Calculate odometry from encoder ticks and publish."""
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0:
            return

        # Get encoder deltas (thread-safe)
        with self.lock:
            left_delta = self.left_ticks - self.last_left
            right_delta = self.right_ticks - self.last_right
            self.last_left = self.left_ticks
            self.last_right = self.right_ticks

        # Convert tick deltas to distances
        left_dist = left_delta * self.meters_per_tick
        right_dist = right_delta * self.meters_per_tick

        # Differential drive kinematics
        # Linear distance = average of both wheels
        # Angular change = difference / wheel_base
        linear = (left_dist + right_dist) / 2.0
        angular = (right_dist - left_dist) / self.wheel_base

        # Update pose using midpoint integration
        # (more accurate than Euler for curved paths)
        self.theta += angular / 2.0
        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        self.theta += angular / 2.0

        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        # Calculate velocities
        self.vx = linear / dt
        self.vtheta = angular / dt

        # Build and publish odometry message
        self._publish_odom_msg()

    def _publish_odom_msg(self):
        """Construct and publish Odometry message."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity (in robot frame)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vtheta

        # Covariance matrices (rough estimates for small robot)
        # Pose covariance [x, y, z, roll, pitch, yaw]
        # Only x, y, and yaw are relevant for 2D
        odom.pose.covariance[0] = 0.01    # x variance
        odom.pose.covariance[7] = 0.01    # y variance
        odom.pose.covariance[35] = 0.03   # yaw variance

        # Twist covariance [vx, vy, vz, wx, wy, wz]
        odom.twist.covariance[0] = 0.01   # vx variance
        odom.twist.covariance[35] = 0.03  # wz variance

        self.odom_pub.publish(odom)

        # Broadcast TF transform
        if self.publish_tf:
            self._publish_tf(odom)

    def _publish_tf(self, odom: Odometry):
        """Broadcast odom -> base_link transform."""
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def reset_odometry(self):
        """Reset odometry to origin (callable via service if needed)."""
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.left_ticks = 0
            self.right_ticks = 0
            self.last_left = 0
            self.last_right = 0

        self.get_logger().info('Odometry reset to origin')

    def destroy_node(self):
        """Clean shutdown - release GPIO resources."""
        self.get_logger().info('Shutting down encoder odometry...')

        # Cancel callbacks
        for cb in self.callbacks:
            cb.cancel()

        # Close pigpio connection
        if self.pi:
            self.pi.stop()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
