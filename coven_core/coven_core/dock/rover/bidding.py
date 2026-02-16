"""
bidding.py - Task bid cost calculation for COVEN modules

Implements the competitive bidding system where modules calculate
their cost to execute a task. Lower cost = higher priority.

Cost factors:
1. Idle time bonus - prefer idle modules
2. Battery penalty - low battery = higher cost
3. Dock obstruction - penalize paths crossing the dock
4. Task type compatibility - match module capabilities to task requirements

The BidCalculator uses dependency injection for testability without ROS2.

Author: Alexander Shultis
Date: December 2025
"""

import json
import logging
import time
from dataclasses import dataclass
from typing import Optional, Protocol, Dict

from coven_core.config import get_config, BiddingConfig

logger = logging.getLogger(__name__)


class HardwareInterface(Protocol):
    """Protocol for hardware abstraction (dependency injection)."""

    def get_battery_percentage(self) -> float:
        """Return battery level as 0.0-1.0."""
        ...


class PoseProvider(Protocol):
    """Protocol for getting robot position (dependency injection)."""

    def get_position(self) -> tuple[float, float]:
        """Return (x, y) position in meters."""
        ...


@dataclass
class BidFactors:
    """Breakdown of cost calculation factors for transparency/debugging."""

    base_cost: float
    idle_bonus: float = 0.0
    battery_penalty: float = 0.0
    dock_obstruction_penalty: float = 0.0
    task_type_modifier: float = 0.0

    @property
    def total_cost(self) -> float:
        """Calculate total cost from all factors."""
        cost = (
            self.base_cost
            - self.idle_bonus
            + self.battery_penalty
            + self.dock_obstruction_penalty
            + self.task_type_modifier
        )
        return max(0.1, cost)  # Ensure positive cost

    def to_dict(self) -> dict:
        """Convert to dict for logging/debugging."""
        return {
            "base_cost": self.base_cost,
            "idle_bonus": self.idle_bonus,
            "battery_penalty": self.battery_penalty,
            "dock_obstruction": self.dock_obstruction_penalty,
            "task_type_modifier": self.task_type_modifier,
            "total": self.total_cost,
        }


# Module type capabilities: {module_type: {task_type: cost_modifier}}
# Negative = bonus (lower cost), Positive = penalty
MODULE_CAPABILITIES: Dict[str, Dict[str, float]] = {
    "reconrover": {
        "explore": -15,      # Good at exploration
        "map": -10,          # Good at mapping
        "waypoint": 0,       # Neutral for waypoint nav
        "spectral": 100,     # Can't do spectral analysis
        "drill": 100,        # Can't drill
        "cargo": 50,         # Not ideal for cargo
    },
    "spectrometerrover": {
        "explore": 10,       # Can explore, but not ideal
        "map": 10,           # Can map, but not ideal
        "waypoint": 0,       # Neutral
        "spectral": -25,     # PERFECT for spectral
        "drill": 100,        # Can't drill
        "cargo": 50,         # Not for cargo
    },
    "drillrover": {
        "explore": 20,       # Heavy, not great for explore
        "map": 20,           # Not ideal
        "waypoint": 0,       # Can navigate
        "spectral": 100,     # Can't do spectral
        "drill": -25,        # Born for this
        "cargo": 30,         # Somewhat capable
    },
    "cargorover": {
        "explore": 30,       # Too slow for explore
        "map": 30,           # Not ideal
        "waypoint": -5,      # Good at point-to-point
        "spectral": 100,     # Nope
        "drill": 100,        # Nope
        "cargo": -20,        # Cargo hauling champ
    },
}


class BidCalculator:
    """
    Calculate task bid costs for COVEN modules.

    Uses dependency injection for hardware and pose providers,
    enabling unit testing without ROS2.

    Example:
        hw = SimulatedHardware(module_id="test")
        pose = MockPoseProvider(x=1.0, y=2.0)
        calc = BidCalculator(
            module_type="ReconRover",
            hardware=hw,
            pose_provider=pose
        )

        cost = calc.calculate_cost(task_json)
    """

    def __init__(
        self,
        module_type: str,
        hardware: Optional[HardwareInterface] = None,
        pose_provider: Optional[PoseProvider] = None,
        config: Optional[BiddingConfig] = None,
    ):
        """
        Initialize bid calculator.

        Args:
            module_type: Type of module (e.g., "ReconRover", "DrillRover")
            hardware: Hardware interface for battery level
            pose_provider: Pose provider for position-based calculations
            config: Bidding configuration (uses global config if not provided)
        """
        self.module_type = module_type.lower()
        self.hardware = hardware
        self.pose_provider = pose_provider
        self.config = config or get_config().bidding
        self.last_task_complete_time = time.time()

    def calculate_cost(self, task: str) -> float:
        """
        Calculate bid cost for a task.

        Args:
            task: Task description (JSON string or simple string)

        Returns:
            Cost value (lower = better, typically 0-100 range)
        """
        factors = self.calculate_factors(task)
        return factors.total_cost

    def calculate_factors(self, task: str) -> BidFactors:
        """
        Calculate detailed cost breakdown for a task.

        Args:
            task: Task description (JSON string or simple string)

        Returns:
            BidFactors with detailed breakdown
        """
        factors = BidFactors(base_cost=self.config.base_cost)

        # Factor 1: Idle time bonus
        factors.idle_bonus = self._calculate_idle_bonus()

        # Factor 2: Battery penalty
        factors.battery_penalty = self._calculate_battery_penalty()

        # Factor 3: Dock obstruction penalty
        factors.dock_obstruction_penalty = self._calculate_dock_penalty(task)

        # Factor 4: Task type compatibility
        factors.task_type_modifier = self._calculate_task_type_modifier(task)

        logger.debug(f"Bid factors: {factors.to_dict()}")
        return factors

    def _calculate_idle_bonus(self) -> float:
        """Calculate bonus for being idle (prefer idle modules)."""
        idle_time = time.time() - self.last_task_complete_time
        idle_minutes = idle_time / 60.0
        # Scale up to max bonus over configured time period
        bonus = min(idle_minutes / self.config.idle_bonus_minutes, 1.0) * self.config.idle_bonus_max
        return bonus

    def _calculate_battery_penalty(self) -> float:
        """Calculate penalty for low battery."""
        if not self.hardware:
            return 0.0

        try:
            battery_pct = self.hardware.get_battery_percentage()
            # Penalty increases as battery decreases
            penalty = (1.0 - battery_pct) * self.config.battery_penalty_max
            return penalty
        except Exception as e:
            logger.warning(f"Failed to get battery level: {e}")
            return 0.0

    def _calculate_dock_penalty(self, task: str) -> float:
        """
        Calculate penalty for paths that cross through the dock area.

        Penalizes rovers whose waypoints move through the dock,
        with diminishing penalties for later waypoints.

        Args:
            task: Task JSON string

        Returns:
            Penalty value
        """
        if not self.pose_provider:
            return 0.0

        try:
            task_data = json.loads(task)
            waypoints = task_data.get("waypoints", [])
            if not waypoints:
                return 0.0

            x, y = self.pose_provider.get_position()

            # Determine which side of dock we're on
            if abs(x) > abs(y):
                position_side = "east" if x > 0 else "west"
            else:
                position_side = "north" if y > 0 else "south"

            opposites = {
                "north": "south", "south": "north",
                "east": "west", "west": "east"
            }

            # Check first N move waypoints with diminishing penalties
            penalties = self.config.dock_obstruction_penalties
            total_penalty = 0.0
            move_count = 0

            for wp in waypoints:
                if move_count >= len(penalties):
                    break
                if wp.get("type") == "move":
                    direction = wp.get("direction", "").lower()
                    if opposites.get(direction) == position_side:
                        total_penalty += penalties[move_count]
                    move_count += 1

            return total_penalty

        except (json.JSONDecodeError, TypeError, KeyError) as e:
            logger.debug(f"Could not parse waypoints for dock penalty: {e}")
            return 0.0

    def _calculate_task_type_modifier(self, task: str) -> float:
        """
        Calculate cost modifier based on module/task compatibility.

        Args:
            task: Task JSON string

        Returns:
            Cost modifier (negative = bonus, positive = penalty)
        """
        try:
            task_data = json.loads(task)
            task_type = task_data.get("task_type", "").lower()

            if not task_type:
                return 0.0

            capabilities = MODULE_CAPABILITIES.get(self.module_type, {})
            return capabilities.get(task_type, 0.0)

        except (json.JSONDecodeError, TypeError, KeyError):
            return 0.0

    def mark_task_complete(self):
        """Update last task completion time (for idle bonus calculation)."""
        self.last_task_complete_time = time.time()
