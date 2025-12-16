"""
config.py - Centralized configuration for COVEN

Consolidates magic numbers, timing constants, and paths from across the codebase
into typed dataclasses with sensible defaults and environment variable overrides.

CubeRover specs (NASA/Astrobotic):
Reference: https://www.astrobotic.com/lunar-delivery/rovers/cuberover/

Physical:
- Sizes: 2U (20x10x10cm), 4U (20x20x10cm), 6U (30x20x10cm)
- Payload: ~1kg per 10cm cube

Mobility:
- Top Speed: 10 cm/s (0.1 m/s)
- Slope Capability: 30 degrees
- Obstacle Clearance: 15 cm diameter
- Range: Multiple km per lunar day

Power/Comms:
- Power: 0.5W continuous per kg payload
- Bandwidth: 10 kbps per kg payload

Navigation:
- Visual-inertial teleoperation system
- Front/rear wide-angle cameras
- Automatic safeguards

Author: Alexander Shultis
Date: December 2025
"""

import os
from dataclasses import dataclass, field
from typing import Optional
from enum import Enum


@dataclass
class HeartbeatConfig:
    """Heartbeat timing configuration with temporal coupling validation."""

    period: float = 0.8  # seconds between heartbeats
    miss_threshold: int = 3  # missed heartbeats before timeout
    jitter_tolerance: float = 0.2  # acceptable timing variance (seconds)

    def __post_init__(self):
        """Validate temporal coupling: timeout must exceed transmission delays."""
        effective_timeout = self.period * self.miss_threshold
        min_safe_timeout = self.period * 2 + self.jitter_tolerance
        if effective_timeout < min_safe_timeout:
            raise ValueError(
                f"Heartbeat timeout ({effective_timeout}s) is less than "
                f"minimum safe timeout ({min_safe_timeout}s). "
                f"Increase miss_threshold or period."
            )


class CubeRoverSize(Enum):
    """CubeRover form factors (NASA/Astrobotic standard)."""

    SIZE_2U = "2U"  # 20x10x10cm, ~2kg payload
    SIZE_4U = "4U"  # 20x20x10cm, ~4kg payload
    SIZE_6U = "6U"  # 30x20x10cm, ~6kg payload


@dataclass
class CubeRoverConfig:
    """CubeRover physical specifications (NASA/Astrobotic)."""

    size: CubeRoverSize = CubeRoverSize.SIZE_6U

    # Mobility specs (from Astrobotic)
    max_speed: float = 0.1  # m/s (10 cm/s)
    max_slope: float = 30.0  # degrees
    obstacle_clearance: float = 0.15  # meters (15 cm diameter)

    # Physical dimensions (6U default)
    length: float = 0.30  # meters
    width: float = 0.20  # meters
    height: float = 0.10  # meters

    # Power/comms per kg payload
    power_per_kg: float = 0.5  # watts continuous
    bandwidth_per_kg: float = 10.0  # kbps

    @property
    def robot_radius(self) -> float:
        """Approximate collision radius for navigation."""
        return max(self.length, self.width) / 2.0


@dataclass
class NavigationConfig:
    """Navigation and waypoint configuration for CubeRover."""

    # CubeRover speed limits (from CubeRoverConfig)
    max_linear_speed: float = 0.1  # m/s (CubeRover top speed: 10 cm/s)
    max_angular_speed: float = 0.5  # rad/s

    # Waypoint navigation
    waypoint_detour_multiplier: float = 10.0  # abort if detour > N * original
    waypoint_nav_timeout: float = 60.0  # seconds per waypoint
    waypoint_position_tolerance: float = 0.3  # meters - close enough = arrived

    # Robot physical properties (6U CubeRover: ~30x20x10cm)
    robot_radius: float = 0.15  # meters (max(30,20)/2 = 15cm)

    # Goal tolerances
    xy_goal_tolerance: float = 0.03  # 3cm for docking precision
    yaw_goal_tolerance: float = 0.05  # ~3 degrees

    # Acceleration limits (conservative for CubeRover stability)
    max_linear_accel: float = 0.5  # m/s^2
    max_angular_accel: float = 1.0  # rad/s^2


@dataclass
class ExplorationConfig:
    """Frontier-based exploration configuration."""

    frontier_search_radius: float = 3.0  # meters
    min_frontier_size: int = 10  # minimum cells to consider a frontier
    coverage_threshold: float = 0.80  # target coverage before returning
    timeout: float = 300.0  # seconds - max exploration time
    no_frontier_limit: int = 3  # consecutive failures before giving up
    nav_timeout: float = 60.0  # seconds per navigation goal


@dataclass
class BiddingConfig:
    """Task bidding cost calculation configuration."""

    base_cost: float = 50.0

    # Idle time bonus (longer idle = prefer this module)
    idle_bonus_max: float = 20.0  # max reduction for idle time
    idle_bonus_minutes: float = 5.0  # time to reach max bonus

    # Battery penalty (low battery = higher cost)
    battery_penalty_max: float = 30.0  # max penalty at 0% battery

    # Dock obstruction penalties (diminishing for further waypoints)
    dock_obstruction_penalties: tuple = (25.0, 12.0, 6.0, 3.0)


@dataclass
class TaskConfig:
    """Task execution configuration."""

    timeout: float = 300.0  # seconds - watchdog timer
    ready_announce_delay: float = 3.0  # seconds before announcing ready
    ready_max_retries: int = 5  # max ready announcement attempts
    nav_init_max_retries: int = 3  # max navigation init attempts


@dataclass
class PathConfig:
    """Path configuration with environment variable overrides."""

    workspace: str = field(default_factory=lambda: os.environ.get(
        'COVEN_WORKSPACE',
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ))
    map_storage_dir: str = field(default_factory=lambda: os.path.expanduser(
        os.environ.get('COVEN_MAP_DIR', '~/coven_maps')
    ))
    state_dir: str = field(default_factory=lambda: os.path.expanduser(
        os.environ.get('COVEN_STATE_DIR', '~/.coven')
    ))


@dataclass
class CovenConfig:
    """Top-level configuration container."""

    # Hardware specs
    cuberover: CubeRoverConfig = field(default_factory=CubeRoverConfig)

    # Protocol timing
    heartbeat: HeartbeatConfig = field(default_factory=HeartbeatConfig)

    # Navigation and motion
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    exploration: ExplorationConfig = field(default_factory=ExplorationConfig)

    # Task management
    bidding: BiddingConfig = field(default_factory=BiddingConfig)
    task: TaskConfig = field(default_factory=TaskConfig)

    # Paths and storage
    paths: PathConfig = field(default_factory=PathConfig)


# Global configuration instance
_config: Optional[CovenConfig] = None


def get_config() -> CovenConfig:
    """Get the global configuration instance (lazy initialization)."""
    global _config
    if _config is None:
        _config = CovenConfig()
    return _config


def reset_config():
    """Reset configuration to defaults (useful for testing)."""
    global _config
    _config = None
