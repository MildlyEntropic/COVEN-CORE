"""
velocity_executor.py - Dock-Centric Velocity Executor

Receives velocity commands from the dock and executes them on the rover.
Handles command timeout (safety stop if dock communication lost).

This is the rover-side component of the dock-centric architecture.
The dock runs Nav2 and sends velocity commands; the rover just executes them.

Author: Alexander Shultis
Date: December 2025
"""

import logging
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from coven_core.common import VelocityCommand, velocity_command_decode

logger = logging.getLogger(__name__)


class VelocityExecutor:
    """
    Receives and executes velocity commands from the dock.

    In dock-centric mode, the dock runs Nav2 and sends velocity commands.
    The rover simply executes these without local path planning.

    Includes safety timeout - if no command received within timeout,
    the rover stops moving (graceful degradation on link loss).

    Subscribes to:
        - /coven/velocity_cmd (String - JSON encoded VelocityCommand)

    Publishes to:
        - /{namespace}/cmd_vel (Twist)

    Usage:
        executor = VelocityExecutor(node, "Hermione_Granger")
        # Velocity commands will be applied automatically
    """

    def __init__(
        self,
        node: Node,
        module_id: str,
        default_timeout: float = 0.5,
        safety_check_rate: float = 20.0
    ):
        """
        Initialize the velocity executor.

        Args:
            node: ROS2 node to attach subscriptions/publishers to
            module_id: Unique identifier for this rover
            default_timeout: Default command timeout if not specified in message (seconds)
            safety_check_rate: Rate to check for command timeout (Hz)
        """
        self._node = node
        self._module_id = module_id
        self._default_timeout = default_timeout

        # State tracking
        self._last_cmd_time: float = 0.0
        self._current_timeout: float = default_timeout
        self._is_moving: bool = False
        self._enabled: bool = True

        # Latest velocity command
        self._current_linear_x: float = 0.0
        self._current_angular_z: float = 0.0

        # QoS for velocity commands - reliable
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to velocity commands from dock
        self._cmd_sub = node.create_subscription(
            String,
            '/coven/velocity_cmd',
            self._on_velocity_command,
            reliable_qos
        )

        # Publisher for local cmd_vel
        namespace = module_id
        self._cmd_vel_pub = node.create_publisher(
            Twist,
            f'/{namespace}/cmd_vel',
            10
        )

        # Timer for safety check and continuous command output
        period = 1.0 / safety_check_rate
        self._timer = node.create_timer(period, self._safety_check)

        logger.info(f"[{module_id}] VelocityExecutor initialized, timeout={default_timeout}s")

    def _on_velocity_command(self, msg: String) -> None:
        """Handle incoming velocity command from dock."""
        cmd = velocity_command_decode(msg)
        if cmd is None:
            logger.warning(f"[{self._module_id}] Failed to decode velocity command")
            return

        # Only process commands for this rover
        if cmd.module_id != self._module_id:
            return

        if not self._enabled:
            logger.debug(f"[{self._module_id}] Ignoring velocity command - executor disabled")
            return

        # Update state
        self._current_linear_x = cmd.linear_x
        self._current_angular_z = cmd.angular_z
        self._current_timeout = cmd.timeout if cmd.timeout > 0 else self._default_timeout
        self._last_cmd_time = time.time()
        self._is_moving = (abs(cmd.linear_x) > 0.001 or abs(cmd.angular_z) > 0.001)

        # Immediately publish the command
        self._publish_cmd_vel()

    def _safety_check(self) -> None:
        """
        Check for command timeout and continue publishing.

        If timeout exceeded, stop the rover for safety.
        Otherwise, continue publishing the current velocity.
        """
        if not self._enabled:
            return

        now = time.time()
        time_since_cmd = now - self._last_cmd_time

        if self._is_moving and time_since_cmd > self._current_timeout:
            # Timeout - stop for safety
            logger.warning(
                f"[{self._module_id}] Command timeout ({time_since_cmd:.2f}s > "
                f"{self._current_timeout:.2f}s) - stopping"
            )
            self._current_linear_x = 0.0
            self._current_angular_z = 0.0
            self._is_moving = False

        # Publish current velocity (either active command or stop)
        self._publish_cmd_vel()

    def _publish_cmd_vel(self) -> None:
        """Publish current velocity command to cmd_vel topic."""
        twist = Twist()
        twist.linear.x = self._current_linear_x
        twist.angular.z = self._current_angular_z
        self._cmd_vel_pub.publish(twist)

    def stop(self) -> None:
        """Immediately stop the rover."""
        self._current_linear_x = 0.0
        self._current_angular_z = 0.0
        self._is_moving = False
        self._publish_cmd_vel()
        logger.info(f"[{self._module_id}] Stopped by command")

    def enable(self) -> None:
        """Enable velocity execution."""
        self._enabled = True
        logger.info(f"[{self._module_id}] VelocityExecutor enabled")

    def disable(self) -> None:
        """Disable velocity execution and stop."""
        self._enabled = False
        self.stop()
        logger.info(f"[{self._module_id}] VelocityExecutor disabled")

    def is_moving(self) -> bool:
        """Check if rover is currently moving."""
        return self._is_moving

    def is_enabled(self) -> bool:
        """Check if executor is enabled."""
        return self._enabled

    def get_last_cmd_age(self) -> float:
        """Get time since last velocity command (seconds)."""
        if self._last_cmd_time == 0.0:
            return float('inf')
        return time.time() - self._last_cmd_time

    def get_current_velocity(self) -> tuple:
        """
        Get current velocity command.

        Returns:
            Tuple of (linear_x, angular_z)
        """
        return (self._current_linear_x, self._current_angular_z)

    def shutdown(self) -> None:
        """Clean up resources."""
        self.stop()
        if self._timer is not None:
            self._timer.cancel()
        logger.info(f"[{self._module_id}] VelocityExecutor shutdown")
