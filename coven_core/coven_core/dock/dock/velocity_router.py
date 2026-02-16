"""
velocity_router.py - Dock-Centric Velocity Router

Routes velocity commands from Nav2 (or other sources) to the appropriate
rovers. In dock-centric mode, Nav2 runs on the dock and outputs cmd_vel
which needs to be routed to the correct rover.

Author: Alexander Shultis
Date: December 2025
"""

import logging
import time
from typing import Dict, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from coven_core.common import VelocityCommand, velocity_command_encode

logger = logging.getLogger(__name__)


class VelocityRouter:
    """
    Routes velocity commands to rovers.

    In dock-centric architecture:
    - Nav2 outputs to /{active_rover}/cmd_vel_nav
    - VelocityRouter converts to VelocityCommand and sends to /coven/velocity_cmd
    - Rovers execute the velocity commands

    Can also send direct velocity commands bypassing Nav2.

    Subscribes to (dynamically per rover):
        - /{module_id}/cmd_vel_nav (Twist from Nav2)

    Publishes to:
        - /coven/velocity_cmd (String - JSON encoded VelocityCommand)
    """

    def __init__(
        self,
        node: Node,
        command_timeout: float = 0.5
    ):
        """
        Initialize the velocity router.

        Args:
            node: ROS2 node to attach subscriptions/publishers to
            command_timeout: Timeout for velocity commands (passed to rovers)
        """
        self._node = node
        self._command_timeout = command_timeout

        # Active rover being controlled
        self._active_rover: Optional[str] = None

        # Per-rover Nav2 cmd_vel subscriptions
        self._nav_subs: Dict[str, any] = {}

        # QoS for velocity commands
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for velocity commands to rovers
        self._cmd_pub = node.create_publisher(
            String,
            '/coven/velocity_cmd',
            reliable_qos
        )

        # Track last command time per rover
        self._last_cmd_time: Dict[str, float] = {}

        logger.info("VelocityRouter initialized")

    def register_rover(self, module_id: str) -> None:
        """
        Register a rover for velocity command routing.

        Creates a subscription to Nav2 cmd_vel for this rover.

        Args:
            module_id: Rover ID to register
        """
        if module_id in self._nav_subs:
            logger.debug(f"Rover '{module_id}' already registered in velocity router")
            return

        # Subscribe to Nav2 cmd_vel for this rover
        topic = f'/{module_id}/cmd_vel_nav'
        self._nav_subs[module_id] = self._node.create_subscription(
            Twist,
            topic,
            lambda msg, mid=module_id: self._on_nav_cmd_vel(mid, msg),
            10
        )
        logger.info(f"Registered rover '{module_id}' for velocity routing (topic: {topic})")

    def unregister_rover(self, module_id: str) -> None:
        """
        Unregister a rover from velocity command routing.

        Args:
            module_id: Rover ID to unregister
        """
        if module_id in self._nav_subs:
            # Send stop command first
            self.send_stop(module_id)
            # Remove subscription
            self._node.destroy_subscription(self._nav_subs[module_id])
            del self._nav_subs[module_id]
            logger.info(f"Unregistered rover '{module_id}' from velocity routing")

    def set_active_rover(self, module_id: Optional[str]) -> None:
        """
        Set the active rover for Nav2 control.

        Only the active rover receives Nav2 velocity commands.
        Other rovers can still receive direct commands.

        Args:
            module_id: Rover ID to activate, or None to deactivate all
        """
        if self._active_rover and self._active_rover != module_id:
            # Stop the previously active rover
            self.send_stop(self._active_rover)

        self._active_rover = module_id
        if module_id:
            logger.info(f"Active rover set to '{module_id}'")
        else:
            logger.info("No active rover - Nav2 commands will be ignored")

    def _on_nav_cmd_vel(self, module_id: str, msg: Twist) -> None:
        """Handle Nav2 velocity command for a rover."""
        # Only route to active rover
        if module_id != self._active_rover:
            return

        self.send_velocity(module_id, msg.linear.x, msg.angular.z)

    def send_velocity(
        self,
        module_id: str,
        linear_x: float,
        angular_z: float,
        timeout: Optional[float] = None
    ) -> None:
        """
        Send velocity command to a rover.

        Args:
            module_id: Target rover ID
            linear_x: Linear velocity (m/s)
            angular_z: Angular velocity (rad/s)
            timeout: Command timeout (uses default if not specified)
        """
        now = time.time()
        cmd = VelocityCommand(
            module_id=module_id,
            linear_x=linear_x,
            angular_z=angular_z,
            timestamp=now,
            timeout=timeout if timeout is not None else self._command_timeout
        )

        msg = String()
        msg.data = velocity_command_encode(cmd)
        self._cmd_pub.publish(msg)

        self._last_cmd_time[module_id] = now

    def send_stop(self, module_id: str) -> None:
        """Send stop command to a rover."""
        self.send_velocity(module_id, 0.0, 0.0)
        logger.debug(f"Sent stop command to '{module_id}'")

    def send_stop_all(self) -> None:
        """Send stop command to all registered rovers."""
        for module_id in self._nav_subs:
            self.send_stop(module_id)

    def get_active_rover(self) -> Optional[str]:
        """Get the active rover ID."""
        return self._active_rover

    def get_registered_rovers(self) -> list:
        """Get list of registered rover IDs."""
        return list(self._nav_subs.keys())

    def get_last_cmd_time(self, module_id: str) -> Optional[float]:
        """Get time of last velocity command to a rover."""
        return self._last_cmd_time.get(module_id)

    def shutdown(self) -> None:
        """Clean up resources."""
        # Stop all rovers
        self.send_stop_all()
        logger.info("VelocityRouter shutdown")
