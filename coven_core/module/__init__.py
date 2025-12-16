"""
module - COVEN module components

This package contains the refactored components extracted from module_node.py:
- state_machine: FSM for module lifecycle (BOOT → IDENTIFY → NORMAL → FIELD_OPS)
- bidding: Task bid cost calculation
- navigation: Nav2 wrapper for waypoint and exploration missions

Author: Alexander Shultis
Date: December 2025
"""

from coven_core.module.state_machine import ModuleStateMachine, ModuleState
from coven_core.module.bidding import BidCalculator
from coven_core.module.navigation import ModuleNavigator

__all__ = ['ModuleStateMachine', 'ModuleState', 'BidCalculator', 'ModuleNavigator']
