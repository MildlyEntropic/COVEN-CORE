"""
rover_manager.py - Dock-Centric Rover Manager

Manages registered rovers and their status. Handles rover registration,
tracks rover state, and provides rover lookup for other dock components.

Author: Alexander Shultis
Date: December 2025
"""

import logging
import time
from dataclasses import dataclass
from typing import Dict, Optional, List, Callable

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String

from coven_core.common import (
    SimplifiedModuleState,
    RoverRegistrationAck, DockCommand,
    rover_registration_decode, rover_registration_ack_encode,
    rover_status_decode, dock_command_encode,
)

logger = logging.getLogger(__name__)


@dataclass
class RoverInfo:
    """Information about a registered rover."""
    module_id: str
    module_type: str
    firmware_version: str
    capabilities: List[str]
    namespace: str
    registered_at: float
    last_status_time: float = 0.0
    state: SimplifiedModuleState = SimplifiedModuleState.READY
    battery_level: float = 1.0
    is_moving: bool = False
    last_cmd_age: float = 0.0
    error_msg: str = ""
    # Position tracking (updated from sensor data)
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class RoverManager:
    """
    Manages registered rovers in dock-centric architecture.

    Handles:
    1. Rover registration and acknowledgment
    2. Rover status tracking
    3. Rover lookup for other components
    4. Rover health monitoring (timeout detection)

    Subscribes to:
        - /coven/rover_registration (String - JSON encoded RoverRegistration)
        - /coven/rover_status (String - JSON encoded RoverStatus)

    Publishes to:
        - /coven/registration_ack (String - JSON encoded RoverRegistrationAck)
        - /coven/dock_command (String - JSON encoded DockCommand)
    """

    def __init__(
        self,
        node: Node,
        on_rover_registered: Optional[Callable[[str], None]] = None,
        on_rover_lost: Optional[Callable[[str], None]] = None,
        status_timeout: float = 5.0
    ):
        """
        Initialize the rover manager.

        Args:
            node: ROS2 node to attach subscriptions/publishers to
            on_rover_registered: Callback when rover registers
            on_rover_lost: Callback when rover times out
            status_timeout: Time without status before rover considered lost
        """
        self._node = node
        self._on_rover_registered = on_rover_registered
        self._on_rover_lost = on_rover_lost
        self._status_timeout = status_timeout

        # Registered rovers
        self._rovers: Dict[str, RoverInfo] = {}

        # QoS for reliable communication
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Registration handling
        self._reg_sub = node.create_subscription(
            String,
            '/coven/rover_registration',
            self._on_registration,
            reliable_qos
        )
        self._reg_ack_pub = node.create_publisher(
            String,
            '/coven/registration_ack',
            reliable_qos
        )

        # Status tracking
        self._status_sub = node.create_subscription(
            String,
            '/coven/rover_status',
            self._on_status,
            reliable_qos
        )

        # Command publisher
        self._cmd_pub = node.create_publisher(
            String,
            '/coven/dock_command',
            reliable_qos
        )

        # Health check timer
        self._health_timer = node.create_timer(1.0, self._check_health)

        logger.info("RoverManager initialized")

    def _on_registration(self, msg: String) -> None:
        """Handle rover registration request."""
        reg = rover_registration_decode(msg)
        if reg is None:
            logger.warning("Failed to decode registration message")
            return

        module_id = reg.module_id
        logger.info(f"Registration request from '{module_id}' (type: {reg.module_type})")

        # Check if already registered
        if module_id in self._rovers:
            logger.info(f"Rover '{module_id}' already registered - updating")

        # Create/update rover info
        now = time.time()
        self._rovers[module_id] = RoverInfo(
            module_id=module_id,
            module_type=reg.module_type,
            firmware_version=reg.firmware_version,
            capabilities=reg.capabilities or ["lidar", "odom"],
            namespace=module_id,
            registered_at=now,
            last_status_time=now,
            battery_level=reg.initial_battery
        )

        # Send acknowledgment
        ack = RoverRegistrationAck(
            module_id=module_id,
            accepted=True,
            assigned_namespace=module_id,
            reason=""
        )
        ack_msg = String()
        ack_msg.data = rover_registration_ack_encode(ack)
        self._reg_ack_pub.publish(ack_msg)

        logger.info(f"Rover '{module_id}' registered successfully")

        # Callback
        if self._on_rover_registered:
            self._on_rover_registered(module_id)

    def _on_status(self, msg: String) -> None:
        """Handle rover status update."""
        status = rover_status_decode(msg)
        if status is None:
            return

        module_id = status.module_id
        if module_id not in self._rovers:
            logger.debug(f"Status from unregistered rover '{module_id}' - ignoring")
            return

        # Update rover info
        rover = self._rovers[module_id]
        rover.last_status_time = time.time()
        rover.state = SimplifiedModuleState[status.state]
        rover.battery_level = status.battery_level
        rover.is_moving = status.is_moving
        rover.last_cmd_age = status.last_cmd_age
        rover.error_msg = status.error_msg

    def _check_health(self) -> None:
        """Check for rovers that have timed out."""
        now = time.time()
        lost_rovers = []

        for module_id, rover in self._rovers.items():
            time_since_status = now - rover.last_status_time
            if time_since_status > self._status_timeout:
                lost_rovers.append(module_id)
                logger.warning(
                    f"Rover '{module_id}' lost - no status for "
                    f"{time_since_status:.1f}s (timeout: {self._status_timeout}s)"
                )

        # Remove lost rovers and trigger callbacks
        for module_id in lost_rovers:
            del self._rovers[module_id]
            if self._on_rover_lost:
                self._on_rover_lost(module_id)

    def send_command(self, module_id: str, command: str, parameters: Optional[dict] = None) -> bool:
        """
        Send a command to a rover.

        Args:
            module_id: Target rover ID
            command: Command name ("start", "stop", "return", "shutdown")
            parameters: Optional command parameters

        Returns:
            True if rover exists and command was sent
        """
        if module_id not in self._rovers:
            logger.warning(f"Cannot send command to unknown rover '{module_id}'")
            return False

        cmd = DockCommand(
            module_id=module_id,
            command=command,
            parameters=parameters or {}
        )
        msg = String()
        msg.data = dock_command_encode(cmd)
        self._cmd_pub.publish(msg)

        logger.debug(f"Sent command '{command}' to '{module_id}'")
        return True

    def broadcast_command(self, command: str, parameters: Optional[dict] = None) -> int:
        """
        Send a command to all registered rovers.

        Returns:
            Number of rovers the command was sent to
        """
        count = 0
        for module_id in self._rovers:
            if self.send_command(module_id, command, parameters):
                count += 1
        return count

    def get_rover(self, module_id: str) -> Optional[RoverInfo]:
        """Get rover info by ID."""
        return self._rovers.get(module_id)

    def get_all_rovers(self) -> List[RoverInfo]:
        """Get all registered rovers."""
        return list(self._rovers.values())

    def get_available_rovers(self) -> List[RoverInfo]:
        """Get rovers that are ready for tasks (READY or ACTIVE state)."""
        return [
            rover for rover in self._rovers.values()
            if rover.state in (SimplifiedModuleState.READY, SimplifiedModuleState.ACTIVE)
        ]

    def get_rover_count(self) -> int:
        """Get number of registered rovers."""
        return len(self._rovers)

    def is_rover_registered(self, module_id: str) -> bool:
        """Check if rover is registered."""
        return module_id in self._rovers

    def update_rover_position(self, module_id: str, x: float, y: float, theta: float) -> None:
        """
        Update rover position (called by sensor receiver).

        Args:
            module_id: Rover ID
            x: X position
            y: Y position
            theta: Heading in radians
        """
        if module_id in self._rovers:
            rover = self._rovers[module_id]
            rover.x = x
            rover.y = y
            rover.theta = theta

    def shutdown(self) -> None:
        """Clean up resources."""
        if self._health_timer:
            self._health_timer.cancel()
        logger.info("RoverManager shutdown")
