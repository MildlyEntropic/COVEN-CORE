# SPDX-License-Identifier: MIT
"""
dispatch_tracker.py — Rover status tracking for the frontier dispatcher.

Data types for tracking rover availability, mission history, and
state transitions during exploration dispatch.

Author: Alexander Shultis
Date: December 2025
"""

from typing import Tuple, Optional
from enum import Enum
from dataclasses import dataclass


class DispatchStatus(Enum):
    """Rover availability states."""
    IDLE = "idle"           # At dock, ready for mission
    DEPLOYED = "deployed"   # Out on mission
    RETURNING = "returning" # Coming back
    DOCKED = "docked"       # Just returned, transferring data


@dataclass
class RoverInfo:
    """Tracked information about a rover."""
    module_id: str
    status: DispatchStatus
    last_position: Tuple[float, float]
    current_mission: Optional[str] = None
    missions_completed: int = 0
    registered_time: float = 0.0  # When the rover first registered
    dispatch_time: float = 0.0    # When last dispatched (to detect race conditions)
    previous_status: Optional[DispatchStatus] = None  # Track state transitions
