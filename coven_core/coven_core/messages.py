# SPDX-License-Identifier: MIT
"""
messages.py — COVEN protocol message dataclasses.

All message types for the COVEN docking + module system:
protocol handshake, tasks, bidding, coverage, and dock-centric messages.

Author: Alexander Shultis
Date: September 2025
"""

from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict


# ------------------------
# --- Protocol Messages ---
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


# ------------------------
# --- Waypoints ---
# ------------------------

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


# ------------------------
# --- Mission / Task ---
# ------------------------

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
# --- Dock-Centric Messages ---
# ------------------------
# These messages support the dock-centric architecture where:
# - Rovers are simple sensor/actuator nodes (Pi Zero 2W)
# - Dock runs all heavy computation: SLAM, Nav2, exploration planning
# - Communication is sensor data up, velocity commands down

@dataclass
class RoverRegistration:
    """Rover announces itself to the dock on boot.

    Sent once when rover transitions from BOOT to READY state.
    Dock uses this to track available rovers and their capabilities.
    """

    module_id: str                       # Unique rover identifier
    module_type: str = "lidar_rover"     # Rover type (lidar_rover, camera_rover, etc.)
    firmware_version: str = "1.0.0"      # Firmware/software version
    capabilities: Optional[List[str]] = None  # ["lidar", "odom", "camera", etc.]
    initial_battery: float = 1.0         # Battery level at registration

    def __post_init__(self):
        if self.capabilities is None:
            self.capabilities = ["lidar", "odom"]


@dataclass
class RoverRegistrationAck:
    """Dock acknowledges rover registration.

    Sent in response to RoverRegistration. Provides rover with
    its assigned namespace and any dock-side configuration.
    """

    module_id: str                       # Echoed back for confirmation
    accepted: bool = True                # False if dock rejects rover
    assigned_namespace: str = ""         # Namespace to use (usually module_id)
    reason: str = ""                     # Rejection reason if not accepted


@dataclass
class SensorData:
    """Bundled sensor data from rover to dock.

    Rovers publish this at regular intervals. Contains all sensor
    readings the dock needs for SLAM and navigation. Using a single
    bundled message reduces topic overhead vs separate topics.
    """

    module_id: str
    timestamp: float = 0.0               # ROS time when captured

    # LiDAR scan (compressed)
    scan_ranges: Optional[List[float]] = None   # Range readings
    scan_angle_min: float = 0.0
    scan_angle_max: float = 6.28         # 2*pi for 360 degree
    scan_angle_increment: float = 0.0175  # ~1 degree

    # Odometry
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_theta: float = 0.0              # Heading in radians
    odom_vx: float = 0.0                 # Linear velocity
    odom_vtheta: float = 0.0             # Angular velocity

    # Battery
    battery_level: float = 1.0           # 0.0 - 1.0

    def __post_init__(self):
        if self.scan_ranges is None:
            self.scan_ranges = []


@dataclass
class VelocityCommand:
    """Velocity command from dock to rover.

    Dock computes navigation and sends velocity commands.
    Rover simply executes these without local planning.
    """

    module_id: str                       # Target rover
    linear_x: float = 0.0                # Forward velocity (m/s)
    angular_z: float = 0.0               # Rotation velocity (rad/s)
    timestamp: float = 0.0               # When command was generated
    timeout: float = 0.5                 # Stop if no new command within timeout


@dataclass
class RoverStatusMsg:
    """Simplified status message from rover to dock.

    Lightweight periodic update. More detailed than heartbeat,
    less verbose than full SensorData.
    """

    module_id: str
    state: str = "READY"                 # e.g. "READY", "ACTIVE", "ERROR"
    battery_level: float = 1.0
    is_moving: bool = False              # Currently executing velocity
    last_cmd_age: float = 0.0            # Seconds since last velocity command
    error_msg: str = ""                  # Error details if state is ERROR


@dataclass
class DockCommand:
    """High-level command from dock to rover.

    Used for state transitions and mission control,
    not for continuous velocity commands.
    """

    module_id: str
    command: str                         # "start", "stop", "return", "shutdown"
    parameters: Optional[Dict] = None    # Command-specific parameters

    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}
