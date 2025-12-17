"""
module - COVEN module components

This package contains the refactored components extracted from module_node.py:
- state_machine: FSM for module lifecycle (BOOT → IDENTIFY → NORMAL → FIELD_OPS)
- bidding: Task bid cost calculation
- navigation: Nav2 wrapper for waypoint and exploration missions

Dock-centric architecture components:
- sensor_publisher: Bundles and publishes sensor data to dock
- velocity_executor: Receives and executes velocity commands from dock

Author: Alexander Shultis
Date: December 2025
"""

from coven_core.module.state_machine import ModuleStateMachine, ModuleState
from coven_core.module.bidding import BidCalculator
from coven_core.module.navigation import ModuleNavigator
from coven_core.module.sensor_publisher import SensorPublisher
from coven_core.module.velocity_executor import VelocityExecutor

__all__ = [
    'ModuleStateMachine', 'ModuleState',
    'BidCalculator', 'ModuleNavigator',
    'SensorPublisher', 'VelocityExecutor'
]
