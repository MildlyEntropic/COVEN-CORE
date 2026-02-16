"""
dock.dock - Dock infrastructure sub-components

Components for managing the dock's internal operations:
- rover_manager: Tracks registered rovers and their status
- sensor_receiver: Receives sensor data from rovers via UART
- velocity_router: Routes velocity commands to docked rovers

Author: Alexander Shultis
Date: December 2025
"""

from coven_core.dock.dock.rover_manager import RoverManager
from coven_core.dock.dock.sensor_receiver import SensorReceiver
from coven_core.dock.dock.velocity_router import VelocityRouter

__all__ = ['RoverManager', 'SensorReceiver', 'VelocityRouter']
