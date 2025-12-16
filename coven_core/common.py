"""
common.py — COVEN Phase 1

Shared definitions, message dataclasses, and encode/decode helpers
for the COVEN docking + module FSM system.

Responsibilities:
- Define DockState and ModuleState enums.
- Provide dataclasses for Identify, Verify, Heartbeat, and Task messages.
- Provide JSON encode/decode helpers with error handling.

NOTE: Encode/decode functions now use the generic serializer from
coven_core.serialization. The wrapper functions here maintain backward
compatibility with existing code.

Author: Alexander Shultis
Date: September 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
# --- Standard library ---
import logging
import random
from dataclasses import dataclass
from enum import Enum
from typing import Optional, List, Tuple, Dict

# --- Third-party (ROS2) ---
from std_msgs.msg import String

# --- Generic serializer ---
from coven_core.serialization import encode as _generic_encode, decode as _generic_decode

# Module-level logger
logger = logging.getLogger(__name__)


# ------------------------
# --- Enums ---
# ------------------------
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
    """FSM states for a COVEN module."""

    BOOT = 0
    IDENTIFY = 1
    WAIT_VERIFY = 2
    NORMAL = 3
    REJECTED = 4
    DISCONNECTED = 5
    FIELD_OPS = 6


# ------------------------
# --- ANSI Color Codes ---
# ------------------------
COLOR_GREEN  = "\033[92m"
COLOR_YELLOW = "\033[93m"
COLOR_ORANGE = "\033[38;5;208m"
COLOR_RED    = "\033[91m"
COLOR_RESET  = "\033[0m"


# ------------------------
# --- Witch Naming System ---
# ------------------------
# COVEN: Modules are named after famous witches from mythology, literature, and pop culture
# The dock (coven leader) is named after famous covens or witch gatherings

# Witches for module naming (rovers/modules)
WITCH_NAMES = [
    # Arthurian
    "Morgan_Le_Fay",
    # Greek
    "Hecate",
    "Circe",
    # Celtic
    "Scathach",
    "Morrigan",
    # Germanic
    "Lorelei",
    "Frau_Holle",
    # Finnish
    "Louhi",
    # Slavic
    "Baba_Yaga",
    # West African
    "Mami_Wata",
    # Japanese
    "Princess_Kaguya",
    # Wizard of Oz
    "Elphaba",
    "Glinda",
    # Marvel
    "Wanda_Maximoff",
    "Agatha_Harkness",
    # DC Comics
    "Zatanna_Zatara",
    # Harry Potter
    "Hermione_Granger",
    "Minerva_McGonagall",
    # Sabrina
    "Sabrina_Spellman",
    # Buffy
    "Willow_Rosenberg",
    # Disney
    "Maleficent",
    # Bewitched
    "Endora",
    "Samantha_Stephens",
    # Studio Ghibli
    "Kiki",
    "Yubaba",
    # Little Witch Academia
    "Akko",
    # Witch Watch
    "Nico_Wakatsuki",
    # Star Wars (Dathomir)
    "Mother_Talzin",
    "Old_Daka",
    "Axkva_Min",
]

# Coven names for dock/hub naming (famous witch groups/sisterhoods)
COVEN_NAMES = [
    # Greek
    "The_Graeae",
    "The_Erinyes",
    # Norse
    "The_Norns",
    # Shakespeare
    "The_Weird_Sisters",
    # Hocus Pocus
    "The_Sanderson_Sisters",
    # Scooby-Doo
    "The_Hex_Girls",
    # The Witcher
    "The_Crones",
    # Brave
    "The_Hags_of_Dun_Broch",
    # Dune
    "The_Bene_Gesserit",
    # Stardust
    "The_Lilim",
]

# Track used names to avoid duplicates during runtime
_used_witch_names: set = set()
_used_coven_names: set = set()


def get_witch_name() -> str:
    """
    Get a random available witch name for a module.

    Returns unique names randomly selected from the pool.
    Once all names are used, the pool resets.

    Returns:
        A witch name string (e.g., "Baba_Yaga", "Hermione_Granger")
    """
    global _used_witch_names

    # Get available names (not yet used)
    available = [n for n in WITCH_NAMES if n not in _used_witch_names]

    # If all names used, reset the pool
    if not available:
        _used_witch_names.clear()
        available = WITCH_NAMES.copy()

    # Pick randomly from available
    name = random.choice(available)
    _used_witch_names.add(name)
    return name


def get_coven_name() -> str:
    """
    Get a random available coven name for a dock/hub.

    Returns unique names randomly selected from the pool.
    Once all names are used, the pool resets.

    Returns:
        A coven name string (e.g., "The_Graeae", "The_Norns")
    """
    global _used_coven_names

    # Get available names (not yet used)
    available = [n for n in COVEN_NAMES if n not in _used_coven_names]

    # If all names used, reset the pool
    if not available:
        _used_coven_names.clear()
        available = COVEN_NAMES.copy()

    # Pick randomly from available
    name = random.choice(available)
    _used_coven_names.add(name)
    return name


def reset_naming():
    """Reset naming system (useful for testing)."""
    global _used_witch_names, _used_coven_names
    _used_witch_names.clear()
    _used_coven_names.clear()


# ------------------------
# --- Data Classes ---
# ------------------------
@dataclass
class IdentifyReq:
    """Request for module identification."""

    req_id: str


@dataclass
class IdentifyRep:
    """Response with module identification."""

    req_id: str
    module_id: str
    module_type: str
    fw: str


@dataclass
class VerifyReq:
    """Request for module verification."""

    module_id: str


@dataclass
class VerifyRep:
    """Response with module verification result."""

    module_id: str
    ok: bool
    reason: str


@dataclass
class Heartbeat:
    """Heartbeat message from module to dock."""

    module_id: str
    seq: int


@dataclass
class Waypoint:
    """A single waypoint instruction for navigation."""

    type: str  # "move" or "turn"
    distance: float = 0.0  # meters (for move)
    direction: str = ""  # "north", "south", "east", "west", "forward" (for move)
    angle: float = 0.0  # degrees, positive=clockwise (for turn)

    def __str__(self):
        if self.type == "move":
            return f"{self.distance}m {self.direction}"
        elif self.type == "turn":
            if self.angle < 0:
                return f"turn {abs(self.angle)}° CCW"
            return f"turn {self.angle}° CW"
        return f"unknown waypoint: {self.type}"


@dataclass
class WaypointResult:
    """Result of executing a single waypoint."""

    waypoint_index: int
    success: bool
    actual_distance: float = 0.0  # How far we actually traveled
    blocked_at: Optional[Tuple[float, float]] = None  # (x, y) if blocked
    detour_distance: float = 0.0  # Extra distance traveled avoiding obstacles
    reason: str = ""  # Failure reason if not successful

    def to_dict(self) -> dict:
        """Convert to JSON-serializable dictionary."""
        return {
            "waypoint_index": self.waypoint_index,
            "success": self.success,
            "actual_distance": self.actual_distance,
            "blocked_at": list(self.blocked_at) if self.blocked_at else None,
            "detour_distance": self.detour_distance,
            "reason": self.reason
        }


@dataclass
class MissionRequest:
    """Top-level mission request from user to dock.

    Supports three modes:
    - task="explore" with waypoints: Waypoint-based exploration
    - task="coverage" with coverage_goal: Autonomous coverage exploration
    - task="explore" without waypoints: Legacy frontier exploration (single rover)
    """

    task: str  # "explore", "coverage", or other task types
    waypoints: Optional[List['Waypoint']] = None  # List of Waypoint objects (waypoint mode)
    coverage_goal: Optional['CoverageGoal'] = None  # Coverage exploration goal (coverage mode)
    return_to_dock: bool = True  # Whether to return after completing mission

    def __post_init__(self):
        if self.waypoints is None:
            self.waypoints = []

    def is_coverage_mission(self) -> bool:
        """Check if this is a coverage-based mission."""
        return self.task == "coverage" and self.coverage_goal is not None

    def is_waypoint_mission(self) -> bool:
        """Check if this is a waypoint-based mission."""
        return self.task == "explore" and len(self.waypoints) > 0


@dataclass
class TaskReq:
    """Request from dock to module to perform a task."""

    module_id: str
    task: str


@dataclass
class TaskAck:
    """Module acknowledgment: accepted/rejected task assignment."""

    module_id: str
    accepted: bool
    reason: str = ""


@dataclass
class TaskStart:
    """Module notification to dock that task has begun."""

    module_id: str
    task: str


@dataclass
class TaskComplete:
    """Module notification to dock that task is complete."""

    module_id: str
    task: str
    success: bool = True
    note: str = ""
    map_data: str = ""  # Base64-encoded gzipped map file (PGM)
    map_yaml: str = ""  # Base64-encoded map metadata (YAML)
    exploration_metrics: dict = None  # Area, duration, distance, etc.

    def __post_init__(self):
        if self.exploration_metrics is None:
            self.exploration_metrics = {}


# ------------------------
# --- Bidding System ---
# ------------------------
@dataclass
class BidNotice:
    """Dock broadcasts this to solicit bids from available modules."""

    task_id: str       # Unique ID for this task auction
    task: str          # Task description (e.g., "explore_zone_a")
    deadline: float = 2.0  # Seconds to respond with bid (default: 2s)


@dataclass
class BidProposal:
    """Module response to a BidNotice with cost estimate."""

    task_id: str       # Must match BidNotice.task_id
    module_id: str     # Bidding module
    cost: float = 999.0  # Lower = better (default: high cost if not specified)
    can_execute: bool = True  # False if module cannot execute this task
    reason: str = ""   # Explanation if can_execute is False


# ------------------------
# --- Coverage Exploration ---
# ------------------------
@dataclass
class CoverageGoal:
    """High-level coverage exploration goal sent from dock to rover.

    Instead of explicit waypoints, the rover autonomously explores
    using frontier-based exploration until coverage target is met
    or constraints are hit (time, battery).
    """

    target_coverage: float = 0.95  # 0.0-1.0 target coverage fraction
    sector: Optional[str] = None   # "NE", "NW", "SE", "SW", or None for all
    sector_bounds: Optional[Tuple[float, float, float, float]] = None  # (x_min, y_min, x_max, y_max)
    max_exploration_time: float = 300.0  # seconds before mandatory return
    return_on_low_battery: bool = True   # honor battery threshold
    battery_return_threshold: float = 0.20  # return when battery < 20%


@dataclass
class CoverageStatus:
    """Periodic status update during coverage exploration.

    Sent from rover to dock at regular intervals to report progress.
    """

    module_id: str
    current_coverage: float = 0.0      # 0.0-1.0 cells explored / total in sector
    battery_remaining: float = 1.0     # 0.0-1.0
    distance_traveled: float = 0.0     # meters since mission start
    frontiers_remaining: int = 0       # number of unexplored frontiers
    returning_to_dock: bool = False    # True if heading back
    reason: str = ""                   # Why returning (battery, coverage, timeout, no_frontiers)


@dataclass
class BatteryConfig:
    """Battery simulation configuration."""

    initial_level: float = 1.0           # Start at 100%
    drain_per_meter: float = 0.005       # 0.5% per meter traveled
    drain_per_second_idle: float = 0.0001  # 0.01% per second when idle
    return_threshold: float = 0.20       # Return when below 20%
    critical_threshold: float = 0.05     # Emergency stop below 5%
    recharge_rate: float = 0.10          # 10% per second at dock


@dataclass
class CoverageConfig:
    """Coverage exploration configuration."""

    default_target: float = 0.95         # 95% coverage target
    max_mission_time: float = 600.0      # 10 minutes max
    status_update_interval: float = 5.0  # Update every 5 seconds
    sector_overlap: float = 0.10         # 10% overlap between sectors
    min_sector_size: float = 25.0        # Minimum 25 sq meters per sector


@dataclass
class Sector:
    """A sector of the exploration area assigned to a rover."""

    name: str                            # "NE", "NW", "SE", "SW", or custom
    bounds: Tuple[float, float, float, float]  # (x_min, y_min, x_max, y_max)
    assigned_to: Optional[str] = None    # module_id if assigned
    coverage: float = 0.0                # 0.0-1.0 current coverage


@dataclass
class CoverageMissionComplete:
    """Final report when coverage mission ends."""

    success: bool = True
    total_coverage: float = 0.0          # Final global coverage achieved
    target_coverage: float = 0.95        # What we were trying to achieve
    total_time: float = 0.0              # Total mission time in seconds
    rovers_dispatched: int = 0           # How many rovers participated
    dispatch_cycles: int = 0             # How many re-dispatch cycles


# ------------------------
# --- Encode / Decode ---
# ------------------------
# These wrapper functions use the generic serializer from coven_core.serialization
# for consistency. They maintain backward compatibility with existing code.

# IDENTIFY
def ident_req_encode(req: IdentifyReq) -> str:
    """Encode IdentifyReq to JSON string."""
    return _generic_encode(req)


def ident_req_decode(msg: String) -> Optional[IdentifyReq]:
    """Decode IdentifyReq from ROS String message."""
    return _generic_decode(msg, IdentifyReq)


def ident_rep_encode(rep: IdentifyRep) -> str:
    """Encode IdentifyRep to JSON string."""
    return _generic_encode(rep)


def ident_rep_decode(msg: String) -> Optional[IdentifyRep]:
    """Decode IdentifyRep from ROS String message."""
    return _generic_decode(msg, IdentifyRep)


# VERIFY
def verify_req_encode(req: VerifyReq) -> str:
    """Encode VerifyReq to JSON string."""
    return _generic_encode(req)


def verify_req_decode(msg: String) -> Optional[VerifyReq]:
    """Decode VerifyReq from ROS String message."""
    return _generic_decode(msg, VerifyReq)


def verify_rep_encode(rep: VerifyRep) -> str:
    """Encode VerifyRep to JSON string."""
    return _generic_encode(rep)


def verify_rep_decode(msg: String) -> Optional[VerifyRep]:
    """Decode VerifyRep from ROS String message."""
    return _generic_decode(msg, VerifyRep)


# HEARTBEAT
def hb_encode(hb: Heartbeat) -> str:
    """Encode Heartbeat to JSON string."""
    return _generic_encode(hb)


def hb_decode(msg: String) -> Optional[Heartbeat]:
    """Decode Heartbeat from ROS String message."""
    return _generic_decode(msg, Heartbeat)


# MISSION_REQ
def mission_req_encode(req: MissionRequest) -> str:
    """Encode MissionRequest to JSON string."""
    return _generic_encode(req)


def mission_req_decode(msg: String) -> Optional[MissionRequest]:
    """Decode MissionRequest from ROS String message."""
    return _generic_decode(msg, MissionRequest)


# TASK_REQ
def task_req_encode(req: TaskReq) -> str:
    """Encode TaskReq to JSON string."""
    return _generic_encode(req)


def task_req_decode(msg: String) -> Optional[TaskReq]:
    """Decode TaskReq from ROS String message."""
    return _generic_decode(msg, TaskReq)


# TASK_ACK
def task_ack_encode(ack: TaskAck) -> str:
    """Encode TaskAck to JSON string."""
    return _generic_encode(ack)


def task_ack_decode(msg: String) -> Optional[TaskAck]:
    """Decode TaskAck from ROS String message."""
    return _generic_decode(msg, TaskAck)


# TASK_START
def task_start_encode(ts: TaskStart) -> str:
    """Encode TaskStart to JSON string."""
    return _generic_encode(ts)


def task_start_decode(msg: String) -> Optional[TaskStart]:
    """Decode TaskStart from ROS String message."""
    return _generic_decode(msg, TaskStart)


# TASK_COMPLETE
def task_complete_encode(tc: TaskComplete) -> str:
    """Encode TaskComplete to JSON string."""
    return _generic_encode(tc)


def task_complete_decode(msg: String) -> Optional[TaskComplete]:
    """Decode TaskComplete from ROS String message."""
    return _generic_decode(msg, TaskComplete)


# BID_NOTICE
def bid_notice_encode(bn: BidNotice) -> str:
    """Encode BidNotice to JSON string."""
    return _generic_encode(bn)


def bid_notice_decode(msg: String) -> Optional[BidNotice]:
    """Decode BidNotice from ROS String message."""
    return _generic_decode(msg, BidNotice)


# BID_PROPOSAL
def bid_proposal_encode(bp: BidProposal) -> str:
    """Encode BidProposal to JSON string."""
    return _generic_encode(bp)


def bid_proposal_decode(msg: String) -> Optional[BidProposal]:
    """Decode BidProposal from ROS String message."""
    return _generic_decode(msg, BidProposal)


# COVERAGE_GOAL
def coverage_goal_encode(cg: CoverageGoal) -> str:
    """Encode CoverageGoal to JSON string."""
    return _generic_encode(cg)


def coverage_goal_decode(msg: String) -> Optional[CoverageGoal]:
    """Decode CoverageGoal from ROS String message."""
    return _generic_decode(msg, CoverageGoal)


# COVERAGE_STATUS
def coverage_status_encode(cs: CoverageStatus) -> str:
    """Encode CoverageStatus to JSON string."""
    return _generic_encode(cs)


def coverage_status_decode(msg: String) -> Optional[CoverageStatus]:
    """Decode CoverageStatus from ROS String message."""
    return _generic_decode(msg, CoverageStatus)


# COVERAGE_MISSION_COMPLETE
def coverage_mission_complete_encode(cmc: CoverageMissionComplete) -> str:
    """Encode CoverageMissionComplete to JSON string."""
    return _generic_encode(cmc)


def coverage_mission_complete_decode(msg: String) -> Optional[CoverageMissionComplete]:
    """Decode CoverageMissionComplete from ROS String message."""
    return _generic_decode(msg, CoverageMissionComplete)