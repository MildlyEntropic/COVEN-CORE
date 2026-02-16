"""
simplified_module_node.py - Dock-Centric Rover Node

!!! DEPRECATED !!!
This file is deprecated and no longer used. Rovers now run Rust firmware
(see lightweight_rover_rs/) instead of Python/ROS2. This file is kept for
reference only and will be removed in a future cleanup.

The Rust implementation provides:
- Lower memory footprint (no ROS2 runtime)
- Better real-time performance
- Direct hardware access via rppal
- UART communication with dock (no WiFi/TCP)

See: lightweight_rover_rs/src/state.rs for the active rover state machine.
!!! DEPRECATED !!!

Original description:
A lightweight rover node for the dock-centric architecture. This replaces
the heavy module_node.py which ran SLAM, Nav2, and exploration locally.

In dock-centric mode:
- Rover: Sensors + Velocity execution + Battery reporting (~500 lines)
- Dock: SLAM + Nav2 + Exploration + Coverage coordination (~2000 lines)

This node handles:
1. Registration with dock on startup
2. Publishing sensor data (LiDAR, odometry, battery)
3. Executing velocity commands from dock
4. Reporting status back to dock

FSM: BOOT → READY → ACTIVE → (ERROR)
- BOOT: Starting up, registering with dock
- READY: Registered, awaiting commands
- ACTIVE: Executing velocity commands
- ERROR: Error state, needs intervention

Author: Alexander Shultis
Date: December 2025
"""

import logging
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String

from coven_core.common import (
    SimplifiedModuleState,
    RoverRegistration, RoverRegistrationAck,
    RoverStatus, DockCommand,
    rover_registration_encode, rover_registration_ack_decode,
    rover_status_encode, dock_command_decode,
    get_witch_name,
)
from coven_core.module.sensor_publisher import SensorPublisher
from coven_core.module.velocity_executor import VelocityExecutor

logger = logging.getLogger(__name__)


class SimplifiedModuleNode(Node):
    """
    Lightweight rover node for dock-centric architecture.

    This is a minimal node that:
    1. Registers with dock on startup
    2. Publishes sensor data to dock
    3. Executes velocity commands from dock
    4. Reports status back to dock

    All heavy computation (SLAM, Nav2, exploration) runs on the dock.
    """

    def __init__(self, module_id: Optional[str] = None):
        """
        Initialize the simplified module node.

        Args:
            module_id: Optional module ID. If None, generates a random witch name.
        """
        # Generate module ID if not provided
        if module_id is None:
            module_id = get_witch_name()

        super().__init__(f'coven_module_{module_id}')

        self._module_id = module_id
        self._state = SimplifiedModuleState.BOOT
        self._registered = False
        self._registration_attempts = 0
        self._max_registration_attempts = 10
        self._last_error = ""

        # Battery simulation (same as original module_node)
        self._battery_level = 1.0
        self._battery_drain_per_meter = 0.005
        self._battery_drain_per_second = 0.0001
        self._last_position = (0.0, 0.0)

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Registration publisher and subscriber
        self._reg_pub = self.create_publisher(
            String,
            '/coven/rover_registration',
            reliable_qos
        )
        self._reg_ack_sub = self.create_subscription(
            String,
            '/coven/registration_ack',
            self._on_registration_ack,
            reliable_qos
        )

        # Status publisher
        self._status_pub = self.create_publisher(
            String,
            '/coven/rover_status',
            reliable_qos
        )

        # Dock command subscriber
        self._cmd_sub = self.create_subscription(
            String,
            '/coven/dock_command',
            self._on_dock_command,
            reliable_qos
        )

        # Create sensor publisher and velocity executor
        self._sensor_publisher = SensorPublisher(
            self,
            module_id,
            publish_rate=10.0,
            battery_callback=self.get_battery_level
        )
        self._velocity_executor = VelocityExecutor(
            self,
            module_id,
            default_timeout=0.5
        )

        # Disable velocity until registered
        self._velocity_executor.disable()

        # Timers
        self._registration_timer = self.create_timer(1.0, self._registration_tick)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self._battery_timer = self.create_timer(1.0, self._update_battery)

        self.get_logger().info(f"SimplifiedModuleNode '{module_id}' starting in BOOT state")

    @property
    def module_id(self) -> str:
        """Get module ID."""
        return self._module_id

    @property
    def state(self) -> SimplifiedModuleState:
        """Get current state."""
        return self._state

    def _set_state(self, new_state: SimplifiedModuleState) -> None:
        """Transition to a new state."""
        if new_state != self._state:
            old_state = self._state
            self._state = new_state
            self.get_logger().info(f"State: {old_state.name} → {new_state.name}")

    def _registration_tick(self) -> None:
        """Periodically attempt registration until successful."""
        if self._registered:
            return

        if self._registration_attempts >= self._max_registration_attempts:
            self._last_error = "Registration failed after max attempts"
            self._set_state(SimplifiedModuleState.ERROR)
            self.get_logger().error(self._last_error)
            return

        # Send registration request
        reg = RoverRegistration(
            module_id=self._module_id,
            module_type="lidar_rover",
            firmware_version="2.0.0",
            capabilities=["lidar", "odom"],
            initial_battery=self._battery_level
        )
        msg = String()
        msg.data = rover_registration_encode(reg)
        self._reg_pub.publish(msg)

        self._registration_attempts += 1
        self.get_logger().debug(
            f"Registration attempt {self._registration_attempts}/{self._max_registration_attempts}"
        )

    def _on_registration_ack(self, msg: String) -> None:
        """Handle registration acknowledgment from dock."""
        ack = rover_registration_ack_decode(msg)
        if ack is None:
            return

        # Only process acks for this module
        if ack.module_id != self._module_id:
            return

        if ack.accepted:
            self._registered = True
            self._set_state(SimplifiedModuleState.READY)
            self._velocity_executor.enable()
            self.get_logger().info(
                f"Registered with dock, namespace: {ack.assigned_namespace}"
            )
        else:
            self._last_error = f"Registration rejected: {ack.reason}"
            self._set_state(SimplifiedModuleState.ERROR)
            self.get_logger().error(self._last_error)

    def _on_dock_command(self, msg: String) -> None:
        """Handle high-level commands from dock."""
        cmd = dock_command_decode(msg)
        if cmd is None:
            return

        # Only process commands for this module
        if cmd.module_id != self._module_id:
            return

        self.get_logger().info(f"Received command: {cmd.command}")

        if cmd.command == "start":
            self._set_state(SimplifiedModuleState.ACTIVE)
            self._velocity_executor.enable()

        elif cmd.command == "stop":
            self._velocity_executor.stop()
            self._set_state(SimplifiedModuleState.READY)

        elif cmd.command == "return":
            # In dock-centric mode, dock handles return navigation
            # Rover just acknowledges and continues executing commands
            self.get_logger().info("Return command acknowledged - awaiting velocity commands")

        elif cmd.command == "shutdown":
            self._velocity_executor.disable()
            self.get_logger().info("Shutdown command received - shutting down")
            self._set_state(SimplifiedModuleState.BOOT)
            # In real deployment, could trigger system shutdown

        else:
            self.get_logger().warning(f"Unknown command: {cmd.command}")

    def _publish_status(self) -> None:
        """Publish status to dock."""
        status = RoverStatus(
            module_id=self._module_id,
            state=self._state.name,
            battery_level=self._battery_level,
            is_moving=self._velocity_executor.is_moving(),
            last_cmd_age=self._velocity_executor.get_last_cmd_age(),
            error_msg=self._last_error
        )
        msg = String()
        msg.data = rover_status_encode(status)
        self._status_pub.publish(msg)

    def _update_battery(self) -> None:
        """Update simulated battery level."""
        # Drain based on time
        self._battery_level -= self._battery_drain_per_second

        # Drain based on distance traveled
        if self._sensor_publisher.has_odom():
            x, y, _ = self._sensor_publisher.get_latest_pose()
            dx = x - self._last_position[0]
            dy = y - self._last_position[1]
            distance = (dx * dx + dy * dy) ** 0.5
            self._battery_level -= distance * self._battery_drain_per_meter
            self._last_position = (x, y)

        # Clamp to valid range
        self._battery_level = max(0.0, min(1.0, self._battery_level))

        # Check for critical battery
        if self._battery_level < 0.05:
            self._velocity_executor.stop()
            self._last_error = "Critical battery level"
            self._set_state(SimplifiedModuleState.ERROR)
            self.get_logger().error("Critical battery - stopped for safety")

    def get_battery_level(self) -> float:
        """Get current battery level (0.0-1.0)."""
        return self._battery_level

    def set_battery_level(self, level: float) -> None:
        """Set battery level (for charging simulation)."""
        self._battery_level = max(0.0, min(1.0, level))

    def is_ready(self) -> bool:
        """Check if rover is ready (sensors available and registered)."""
        return self._registered and self._sensor_publisher.is_ready()

    def shutdown(self) -> None:
        """Clean up resources."""
        self._velocity_executor.shutdown()
        self._sensor_publisher.shutdown()
        self.get_logger().info("SimplifiedModuleNode shutdown complete")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Create a temporary node to get the module_id parameter
    temp_node = rclpy.create_node('_temp_param_reader')
    temp_node.declare_parameter('module_id', '')
    module_id = temp_node.get_parameter('module_id').get_parameter_value().string_value
    temp_node.destroy_node()

    # Create the actual node with the module_id (or None for random)
    node = SimplifiedModuleNode(module_id=module_id if module_id else None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
