"""
simplified_dock_node.py - Dock-Centric Dock Node

A dock node optimized for the dock-centric architecture. This works
alongside the existing dock_node.py but uses the new dock submodules
for rover management, sensor reception, and velocity routing.

In dock-centric mode:
- Dock: SLAM + Nav2 + Exploration + Coverage coordination
- Rovers: Sensors + Velocity execution only

This node orchestrates:
1. Rover registration and management
2. Sensor data reception and republishing
3. Velocity command routing to rovers
4. (Future) SLAM coordination with multi-rover fusion
5. (Future) Exploration planning and task dispatch

Author: Alexander Shultis
Date: December 2025
"""

import logging
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String

from coven_core.common import (
    SimplifiedModuleState, SensorData,
    CoverageGoal, CoverageMissionComplete,
    coverage_goal_encode,
    get_coven_name,
)
from coven_core.dock.rover_manager import RoverManager, RoverInfo
from coven_core.dock.sensor_receiver import SensorReceiver
from coven_core.dock.velocity_router import VelocityRouter

logger = logging.getLogger(__name__)


class SimplifiedDockNode(Node):
    """
    Dock node for dock-centric architecture.

    Manages rovers, receives sensor data, and routes velocity commands.
    Designed to work with simplified_module_node.py on the rover side.
    """

    def __init__(self, dock_id: Optional[str] = None):
        """
        Initialize the simplified dock node.

        Args:
            dock_id: Optional dock ID. If None, generates a random coven name.
        """
        # Generate dock ID if not provided
        if dock_id is None:
            dock_id = get_coven_name()

        super().__init__(f'coven_dock_{dock_id}')

        self._dock_id = dock_id

        # Initialize submodules
        self._rover_manager = RoverManager(
            self,
            on_rover_registered=self._on_rover_registered,
            on_rover_lost=self._on_rover_lost,
            status_timeout=5.0
        )
        self._sensor_receiver = SensorReceiver(
            self,
            on_sensor_data=self._on_sensor_data
        )
        self._velocity_router = VelocityRouter(
            self,
            command_timeout=0.5
        )

        # Mission state
        self._active_mission = False
        self._coverage_target = 0.0
        self._current_coverage = 0.0

        # Status timer
        self._status_timer = self.create_timer(5.0, self._print_status)

        self.get_logger().info(f"SimplifiedDockNode '{dock_id}' initialized")
        self.get_logger().info("Waiting for rover registrations...")

    @property
    def dock_id(self) -> str:
        """Get dock ID."""
        return self._dock_id

    def _on_rover_registered(self, module_id: str) -> None:
        """Handle rover registration."""
        self.get_logger().info(f"Rover '{module_id}' registered")

        # Register for velocity routing
        self._velocity_router.register_rover(module_id)

        # If this is the first rover and no active rover, set it active
        if self._velocity_router.get_active_rover() is None:
            self._velocity_router.set_active_rover(module_id)
            self.get_logger().info(f"Set '{module_id}' as active rover")

    def _on_rover_lost(self, module_id: str) -> None:
        """Handle rover loss."""
        self.get_logger().warning(f"Rover '{module_id}' lost")

        # Unregister from velocity routing
        self._velocity_router.unregister_rover(module_id)

        # If this was the active rover, switch to another
        if self._velocity_router.get_active_rover() == module_id:
            available = self._rover_manager.get_available_rovers()
            if available:
                new_active = available[0].module_id
                self._velocity_router.set_active_rover(new_active)
                self.get_logger().info(f"Switched active rover to '{new_active}'")
            else:
                self._velocity_router.set_active_rover(None)
                self.get_logger().warning("No available rovers")

    def _on_sensor_data(self, data: SensorData) -> None:
        """Handle sensor data from rover."""
        # Update rover position in manager
        self._rover_manager.update_rover_position(
            data.module_id,
            data.odom_x,
            data.odom_y,
            data.odom_theta
        )

        # Future: feed to SLAM, update coverage, etc.

    def _print_status(self) -> None:
        """Periodically print dock status."""
        rovers = self._rover_manager.get_all_rovers()
        if not rovers:
            return

        self.get_logger().info(f"--- Dock Status ({len(rovers)} rovers) ---")
        for rover in rovers:
            pos = f"({rover.x:.2f}, {rover.y:.2f})"
            self.get_logger().info(
                f"  {rover.module_id}: {rover.state.name}, "
                f"battery={rover.battery_level:.0%}, pos={pos}"
            )

    def get_rover_count(self) -> int:
        """Get number of registered rovers."""
        return self._rover_manager.get_rover_count()

    def get_rovers(self) -> List[RoverInfo]:
        """Get all registered rovers."""
        return self._rover_manager.get_all_rovers()

    def get_active_rover(self) -> Optional[str]:
        """Get the active rover ID."""
        return self._velocity_router.get_active_rover()

    def set_active_rover(self, module_id: str) -> bool:
        """
        Set the active rover.

        Args:
            module_id: Rover ID to activate

        Returns:
            True if rover exists and was activated
        """
        if not self._rover_manager.is_rover_registered(module_id):
            self.get_logger().warning(f"Cannot activate unknown rover '{module_id}'")
            return False

        self._velocity_router.set_active_rover(module_id)
        return True

    def send_rover_command(self, module_id: str, command: str, params: dict = None) -> bool:
        """
        Send a command to a rover.

        Args:
            module_id: Target rover ID
            command: Command name
            params: Command parameters

        Returns:
            True if command was sent
        """
        return self._rover_manager.send_command(module_id, command, params)

    def stop_all_rovers(self) -> None:
        """Stop all rovers."""
        self._velocity_router.send_stop_all()
        self._rover_manager.broadcast_command("stop")
        self.get_logger().info("Stopped all rovers")

    def shutdown(self) -> None:
        """Clean up resources."""
        self.stop_all_rovers()
        self._rover_manager.shutdown()
        self._sensor_receiver.shutdown()
        self._velocity_router.shutdown()
        self.get_logger().info("SimplifiedDockNode shutdown complete")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = SimplifiedDockNode()

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
