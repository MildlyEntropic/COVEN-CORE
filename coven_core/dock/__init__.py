"""
dock - COVEN dock-centric submodules

This package contains dock-side components for the dock-centric architecture:
- rover_manager: Tracks registered rovers and their status
- sensor_receiver: Receives sensor data from rovers
- velocity_router: Routes velocity commands to rovers
- slam_coordinator: Manages SLAM with multi-rover sensor fusion (future)
- exploration_planner: Frontier-based exploration planning (future)

Author: Alexander Shultis
Date: December 2025
"""

from coven_core.dock.rover_manager import RoverManager
from coven_core.dock.sensor_receiver import SensorReceiver
from coven_core.dock.velocity_router import VelocityRouter

__all__ = ['RoverManager', 'SensorReceiver', 'VelocityRouter']
