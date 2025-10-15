"""
dock_and_module.launch.py — COVEN Phase 1

ROS2 launch file that starts both the Dock node and a single Module node.
Used for integration testing of the full FSM lifecycle:

    Dock: IDLE → DETECTED → IDENTIFY → VERIFY → ENABLED → NORMAL
    Module: BOOT → IDENTIFY → WAIT_VERIFY → NORMAL

Responsibilities:
- Define a launch description that includes the Dock and one Module node.
- Provide a single entrypoint for testing Dock ↔ Module handshakes.

Author: Alexander Shultis
Date: September 2025
"""

from launch_ros.actions import Node

# ------------------------
# --- Imports ---
# ------------------------
# --- Third-party (ROS2) ---
from launch import LaunchDescription


# ------------------------
# --- Launch Description ---
# ------------------------
def generate_launch_description():
    """Generate a LaunchDescription including Dock and Module nodes."""
    return LaunchDescription(
        [
            Node(package="coven_core", executable="dock", name="dock"),
            Node(package="coven_core", executable="module", name="module"),
        ]
    )
