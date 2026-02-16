"""
enums.py — COVEN state enums and ANSI color codes.

Author: Alexander Shultis
Date: September 2025
"""

from enum import Enum


class DockState(Enum):
    """FSM states for the docking hub."""

    IDLE = 0
    DETECTED = 1
    IDENTIFY = 2
    VERIFY = 3
    ENABLED = 4
    NORMAL = 5
    REJECTED = 6


class ModuleState(Enum):
    """FSM states for a COVEN module (legacy - used in smart-rover architecture)."""

    BOOT = 0
    IDENTIFY = 1
    WAIT_VERIFY = 2
    NORMAL = 3
    REJECTED = 4
    DISCONNECTED = 5
    FIELD_OPS = 6


class SimplifiedModuleState(Enum):
    """Simplified FSM states for dock-centric architecture.

    In dock-centric mode, rovers are simple sensor/actuator nodes.
    Heavy computation (SLAM, Nav2, exploration) runs on the dock.
    """

    BOOT = 0      # Starting up, not yet registered
    READY = 1     # Registered with dock, awaiting commands
    ACTIVE = 2    # Executing velocity commands
    ERROR = 3     # Error state, needs intervention


# ANSI color codes for terminal output
COLOR_GREEN  = "\033[92m"
COLOR_YELLOW = "\033[93m"
COLOR_ORANGE = "\033[38;5;208m"
COLOR_RED    = "\033[91m"
COLOR_RESET  = "\033[0m"
