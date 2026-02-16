"""
dock.rover - Dock-side rover management components

These components run on the dock and manage rover lifecycle:
- state_machine: FSM for module states as seen by the dock
- bidding: Task bid cost calculation for auction-based dispatch
- navigation: Nav2 wrapper for waypoint missions (dock-side planning)
- sensor_publisher: Republishes rover sensor data to ROS2 topics
- velocity_executor: Routes velocity commands to docked rovers

Note: Rovers themselves run Rust firmware (rover/).
These components represent the dock's view of each rover.

Author: Alexander Shultis
Date: December 2025
"""

from coven_core.dock.rover.state_machine import ModuleStateMachine, ModuleState
from coven_core.dock.rover.bidding import BidCalculator
from coven_core.dock.rover.navigation import ModuleNavigator
from coven_core.dock.rover.sensor_publisher import SensorPublisher
from coven_core.dock.rover.velocity_executor import VelocityExecutor

__all__ = [
    'ModuleStateMachine', 'ModuleState',
    'BidCalculator', 'ModuleNavigator',
    'SensorPublisher', 'VelocityExecutor'
]
