#!/usr/bin/env python3
"""
spawn_module.py — Spawn visual representation of COVEN modules in Gazebo

This script spawns visual module models in Gazebo at specified positions.
Each module gets spawned at a unique location to prevent clipping.

Author: Alexander Shultis
Date: November 2025
"""

import os
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


class ModuleSpawner(Node):
    """Node to spawn COVEN module visual models in Gazebo."""

    def __init__(self):
        super().__init__('module_spawner')

        # Create client for Gazebo spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def spawn_module(self, module_id: str, x: float, y: float, z: float = 0.1):
        """
        Spawn a module model in Gazebo at the specified position.

        Args:
            module_id: Unique identifier for the module (e.g., "Strix-1")
            x: X position in meters
            y: Y position in meters
            z: Z position in meters (default: 0.1 to sit on ground)
        """
        # Wait for Gazebo spawn service
        self.get_logger().info("Waiting for /spawn_entity service...")
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Gazebo spawn service not available!")
            return False

        # Load SDF model file
        try:
            pkg_path = get_package_share_directory('coven_core')
            sdf_file = os.path.join(pkg_path, 'models', 'module.sdf')

            if not os.path.exists(sdf_file):
                self.get_logger().error(f"SDF file not found: {sdf_file}")
                return False

            with open(sdf_file, 'r') as f:
                sdf_content = f.read()

        except Exception as e:
            self.get_logger().error(f"Failed to load SDF model: {e}")
            return False

        # Create spawn request
        request = SpawnEntity.Request()
        request.name = module_id
        request.xml = sdf_content
        request.robot_namespace = ''
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.reference_frame = 'world'

        # Call spawn service
        self.get_logger().info(f"Spawning module '{module_id}' at ({x:.2f}, {y:.2f}, {z:.2f})")
        future = self.spawn_client.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"✓ Module '{module_id}' spawned successfully")
                return True
            else:
                self.get_logger().error(f"Failed to spawn '{module_id}': {future.result().status_message}")
                return False
        else:
            self.get_logger().error(f"Spawn service call failed for '{module_id}'")
            return False


def main():
    """Main entry point for spawning a single module."""
    rclpy.init()

    # Parse command-line arguments
    if len(sys.argv) < 4:
        print("Usage: spawn_module.py <module_id> <x> <y> [z]")
        print("Example: spawn_module.py Strix-1 0.5 0.0 0.1")
        sys.exit(1)

    module_id = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1

    spawner = ModuleSpawner()
    success = spawner.spawn_module(module_id, x, y, z)

    spawner.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
