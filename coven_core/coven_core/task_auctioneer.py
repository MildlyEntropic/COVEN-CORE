#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
task_auctioneer.py - Auction-based task allocation for COVEN swarm

The dock announces tasks, rovers submit bids, lowest bid wins.

Bid calculation (on rover side, but we need to understand the factors):
- Base bid: 50
- Wrong payload for task: +999999 (effectively infinite, don't assign)
- Low battery (<30%): +25
- Non-critical damage: +5 to +15 depending on severity
- Wrong side of dock (would cross traffic): +5
- Correct payload: -25
- Full battery (>80%): -15
- Easy orientation (facing right way): -5

The auctioneer:
1. Maintains a mission queue
2. When a rover reports IDLE, starts an auction
3. Collects bids from all IDLE rovers (with timeout)
4. Awards mission to lowest bidder
5. Tracks rover status and mission progress

Author: Alexander Shultis
Date: January 2026
"""

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class RoverStatus(Enum):
    """Rover operational status."""
    UNKNOWN = "unknown"
    IDLE = "idle"
    BIDDING = "bidding"  # Participating in an auction
    ASSIGNED = "assigned"  # Won auction, awaiting departure
    ACTIVE = "active"  # On mission
    RETURNING = "returning"


class PayloadType(Enum):
    """Rover payload capabilities."""
    LIDAR = "lidar"
    SPECTROMETER = "spectrometer"
    DRILL = "drill"
    CARGO = "cargo"
    CAMERA = "camera"


class TaskType(Enum):
    """Mission task types."""
    EXPLORE = "explore"  # General exploration (LIDAR)
    SPECTRAL = "spectral"  # Spectral analysis
    SAMPLE = "sample"  # Drilling/sampling
    DELIVER = "deliver"  # Cargo delivery
    SURVEY = "survey"  # Visual survey (camera)


# Payload compatibility matrix: which payloads can do which tasks
PAYLOAD_TASK_COMPATIBILITY: Dict[PayloadType, Dict[TaskType, int]] = {
    PayloadType.LIDAR: {
        TaskType.EXPLORE: -25,  # Perfect match
        TaskType.SPECTRAL: 999999,  # Can't do
        TaskType.SAMPLE: 999999,
        TaskType.DELIVER: 50,  # Can navigate but not ideal
        TaskType.SURVEY: 10,  # Suboptimal
    },
    PayloadType.SPECTROMETER: {
        TaskType.EXPLORE: 10,  # Can explore, not ideal
        TaskType.SPECTRAL: -25,  # Born for this
        TaskType.SAMPLE: 999999,
        TaskType.DELIVER: 50,
        TaskType.SURVEY: 20,
    },
    PayloadType.DRILL: {
        TaskType.EXPLORE: 20,  # Heavy, slow
        TaskType.SPECTRAL: 999999,
        TaskType.SAMPLE: -25,  # Perfect
        TaskType.DELIVER: 30,
        TaskType.SURVEY: 999999,
    },
    PayloadType.CARGO: {
        TaskType.EXPLORE: 30,  # Too slow
        TaskType.SPECTRAL: 999999,
        TaskType.SAMPLE: 999999,
        TaskType.DELIVER: -20,  # Cargo champ
        TaskType.SURVEY: 999999,
    },
    PayloadType.CAMERA: {
        TaskType.EXPLORE: 5,  # Decent
        TaskType.SPECTRAL: 999999,
        TaskType.SAMPLE: 999999,
        TaskType.DELIVER: 50,
        TaskType.SURVEY: -25,  # Perfect
    },
}


@dataclass
class RoverInfo:
    """Tracked information about a connected rover."""
    module_id: str
    status: RoverStatus = RoverStatus.UNKNOWN
    payload: PayloadType = PayloadType.LIDAR  # Default assumption
    capabilities: int = 0x03  # Bitmask (default = ENCODERS + LIDAR for backward compat)
    battery_pct: float = 100.0
    damage_level: int = 0  # 0-3, 0=none, 3=critical
    position: Tuple[float, float] = (0.0, 0.0)
    heading: float = 0.0  # radians
    dock_slot: Optional[int] = None  # Which dock slot rover is in
    current_mission: Optional[str] = None
    missions_completed: int = 0
    last_heartbeat: float = field(default_factory=time.time)

    def calculate_bid(self, task: 'Mission', dock_position: Tuple[float, float]) -> int:
        """
        Calculate this rover's bid for a task.
        Lower is better. Returns very high number if incapable.
        """
        bid = 50  # Base bid

        # Payload compatibility
        payload_mod = PAYLOAD_TASK_COMPATIBILITY.get(
            self.payload, {}
        ).get(task.task_type, 0)
        bid += payload_mod

        # If incompatible, return immediately (don't waste time on other factors)
        if payload_mod >= 999999:
            return 999999

        # Capability penalty: rover without LiDAR can still explore via
        # dead-reckoning but can't contribute scan data for SLAM
        if task.task_type == TaskType.EXPLORE and not (self.capabilities & 0x02):
            bid += 30

        # Battery state
        if self.battery_pct < 30:
            bid += 25  # Low battery penalty
        elif self.battery_pct > 80:
            bid -= 15  # Full battery bonus

        # Damage state
        if self.damage_level == 1:
            bid += 5
        elif self.damage_level == 2:
            bid += 10
        elif self.damage_level >= 3:
            bid += 999999  # Critical damage, don't assign

        # Position/orientation factors
        # Check if task is on opposite side of dock (would cross traffic)
        if task.waypoints:
            target = task.waypoints[0]
            rover_side = self._get_side_of_dock(self.position, dock_position)
            target_side = self._get_side_of_dock(target, dock_position)

            if rover_side and target_side and rover_side != target_side:
                # Would need to cross dock
                bid += 5

            # Check orientation - rovers back into dock, leave forward
            # If target is behind rover, that's bad
            angle_to_target = math.atan2(
                target[1] - self.position[1],
                target[0] - self.position[0]
            )
            angle_diff = abs(self._normalize_angle(angle_to_target - self.heading))
            if angle_diff < math.pi / 4:  # Target is ahead
                bid -= 5  # Easy orientation

        return max(1, bid)  # Minimum bid of 1

    def _get_side_of_dock(
        self, pos: Tuple[float, float], dock: Tuple[float, float]
    ) -> Optional[str]:
        """Determine which side of dock a position is on."""
        dx = pos[0] - dock[0]
        dy = pos[1] - dock[1]

        if abs(dx) > abs(dy):
            return "east" if dx > 0 else "west"
        elif abs(dy) > 0.1:  # Avoid ambiguity near center
            return "north" if dy > 0 else "south"
        return None

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


@dataclass
class Mission:
    """A mission to be auctioned and assigned."""
    mission_id: str
    task_type: TaskType
    waypoints: List[Tuple[float, float]]
    dock_return: Tuple[float, float]  # Where to return after
    priority: int = 0  # Higher = more urgent
    timeout: float = 300.0  # Mission timeout in seconds
    created_at: float = field(default_factory=time.time)

    # Auction state
    auction_start: Optional[float] = None
    bids: Dict[str, int] = field(default_factory=dict)  # module_id -> bid
    assigned_to: Optional[str] = None
    completed: bool = False
    success: Optional[bool] = None

    # Failed auction recovery
    auction_attempts: int = 0
    max_auction_attempts: int = 5  # Drop mission after this many failed auctions

    def to_task_req(self, dock_id: str) -> dict:
        """Convert to TASK_REQ message format."""
        return {
            "dock_id": dock_id,
            "module_id": self.assigned_to or "",
            "task_id": self.mission_id,
            "task": self.task_type.value,
            "waypoints": [
                {"x": wp[0], "y": wp[1], "yaw": 0.0, "tolerance": 0.5}
                for wp in self.waypoints
            ],
            "dock_x": self.dock_return[0],
            "dock_y": self.dock_return[1],
            "coverage_threshold": 0.8,
            "timeout": self.timeout,
        }


class TaskAuctioneer:
    """
    Auction-based task allocation manager.

    Workflow:
    1. Missions are queued (from frontier analysis, user commands, etc.)
    2. When rovers are idle, start auction for next mission
    3. All idle rovers calculate and submit bids
    4. Lowest bid wins, rover receives TASK_REQ
    5. Track mission progress, handle completion/failure
    """

    def __init__(
        self,
        dock_id: str,
        dock_position: Tuple[float, float] = (0.0, 0.0),
        bid_timeout: float = 2.0,  # Seconds to wait for bids
    ):
        self.dock_id = dock_id
        self.dock_position = dock_position
        self.bid_timeout = bid_timeout

        # State
        self.rovers: Dict[str, RoverInfo] = {}
        self.mission_queue: List[Mission] = []
        self.active_missions: Dict[str, Mission] = {}  # mission_id -> Mission
        self.completed_missions: List[Mission] = []

        # Current auction
        self.current_auction: Optional[Mission] = None
        self.auction_lock = threading.Lock()

        # Callbacks
        self._send_task_callback: Optional[callable] = None
        self._on_mission_complete_callback: Optional[callable] = None

    def set_send_task_callback(self, callback: callable):
        """Set callback for sending TASK_REQ to rovers."""
        self._send_task_callback = callback

    def set_mission_complete_callback(self, callback: callable):
        """Set callback for mission completion."""
        self._on_mission_complete_callback = callback

    def register_rover(
        self,
        module_id: str,
        payload: PayloadType = PayloadType.LIDAR,
        **kwargs
    ) -> RoverInfo:
        """Register a new rover or update existing."""
        if module_id not in self.rovers:
            self.rovers[module_id] = RoverInfo(
                module_id=module_id,
                payload=payload,
                **kwargs
            )
            logger.info(f"Registered rover: {module_id} (payload={payload.value}, caps=0x{self.rovers[module_id].capabilities:02x})")
        else:
            # Update existing
            rover = self.rovers[module_id]
            rover.payload = payload
            for key, value in kwargs.items():
                if hasattr(rover, key):
                    setattr(rover, key, value)
            rover.last_heartbeat = time.time()
        return self.rovers[module_id]

    def update_rover_status(
        self,
        module_id: str,
        status: RoverStatus,
        battery_pct: Optional[float] = None,
        position: Optional[Tuple[float, float]] = None,
        heading: Optional[float] = None,
    ):
        """Update rover status from heartbeat."""
        if module_id not in self.rovers:
            self.register_rover(module_id)

        rover = self.rovers[module_id]
        rover.status = status
        rover.last_heartbeat = time.time()

        if battery_pct is not None:
            rover.battery_pct = battery_pct
        if position is not None:
            rover.position = position
        if heading is not None:
            rover.heading = heading

        logger.debug(
            f"Rover {module_id}: status={status.value}, "
            f"battery={rover.battery_pct:.1f}%"
        )

    def queue_mission(self, mission: Mission):
        """Add a mission to the queue."""
        self.mission_queue.append(mission)
        self.mission_queue.sort(key=lambda m: -m.priority)  # Higher priority first
        logger.info(
            f"Queued mission {mission.mission_id} "
            f"(type={mission.task_type.value}, priority={mission.priority})"
        )

    def create_exploration_mission(
        self,
        waypoints: List[Tuple[float, float]],
        priority: int = 0,
    ) -> Mission:
        """Create and queue an exploration mission."""
        mission = Mission(
            mission_id=f"explore_{uuid.uuid4().hex[:8]}",
            task_type=TaskType.EXPLORE,
            waypoints=waypoints,
            dock_return=self.dock_position,
            priority=priority,
        )
        self.queue_mission(mission)
        return mission

    def get_idle_rovers(self) -> List[RoverInfo]:
        """Get list of rovers ready for assignment."""
        return [
            r for r in self.rovers.values()
            if r.status == RoverStatus.IDLE
        ]

    def try_dispatch(self) -> bool:
        """
        Attempt to dispatch next mission to an available rover.
        Returns True if a mission was dispatched.
        """
        with self.auction_lock:
            # Check preconditions
            if not self.mission_queue:
                return False

            idle_rovers = self.get_idle_rovers()
            if not idle_rovers:
                return False

            # Get next mission
            mission = self.mission_queue[0]

            # Run auction
            winner = self._run_auction(mission, idle_rovers)

            if winner is None:
                mission.auction_attempts += 1
                logger.warning(
                    f"No valid bids for mission {mission.mission_id} "
                    f"(attempt {mission.auction_attempts}/{mission.max_auction_attempts})"
                )

                # Remove from front of queue
                self.mission_queue.pop(0)

                # Check if we've exceeded retry limit
                if mission.auction_attempts >= mission.max_auction_attempts:
                    logger.error(
                        f"Mission {mission.mission_id} DROPPED after "
                        f"{mission.auction_attempts} failed auction attempts"
                    )
                    # Don't re-queue - mission is abandoned
                else:
                    # Move mission to back of queue for retry
                    self.mission_queue.append(mission)

                return False

            # Assign mission
            self.mission_queue.pop(0)
            mission.assigned_to = winner.module_id
            winner.status = RoverStatus.ASSIGNED
            winner.current_mission = mission.mission_id

            self.active_missions[mission.mission_id] = mission

            # Send TASK_REQ
            if self._send_task_callback:
                task_req = mission.to_task_req(self.dock_id)
                self._send_task_callback(winner.module_id, task_req)

            logger.info(
                f"Mission {mission.mission_id} assigned to {winner.module_id} "
                f"(bid={mission.bids.get(winner.module_id, '?')})"
            )
            return True

    def _run_auction(
        self,
        mission: Mission,
        bidders: List[RoverInfo]
    ) -> Optional[RoverInfo]:
        """
        Run an auction for a mission.
        All idle rovers calculate bids, lowest wins.
        """
        mission.auction_start = time.time()
        mission.bids.clear()

        logger.info(f"Starting auction for {mission.mission_id}")
        logger.info(f"  Task type: {mission.task_type.value}")
        logger.info(f"  Waypoints: {mission.waypoints}")
        logger.info(f"  Bidders: {[r.module_id for r in bidders]}")

        # Collect bids from all idle rovers
        # In a real distributed system, we'd broadcast and wait for responses
        # Here we calculate locally since rovers are connected to us
        for rover in bidders:
            bid = rover.calculate_bid(mission, self.dock_position)
            mission.bids[rover.module_id] = bid
            logger.info(f"  {rover.module_id} bids: {bid}")

        # Find lowest valid bid
        valid_bids = [
            (module_id, bid)
            for module_id, bid in mission.bids.items()
            if bid < 999999
        ]

        if not valid_bids:
            logger.warning("No valid bids received")
            return None

        # Sort by bid (lowest first), then by module_id for determinism
        valid_bids.sort(key=lambda x: (x[1], x[0]))
        winner_id = valid_bids[0][0]
        winning_bid = valid_bids[0][1]

        logger.info(f"Auction winner: {winner_id} with bid {winning_bid}")

        return self.rovers.get(winner_id)

    def handle_task_complete(
        self,
        module_id: str,
        task_id: str,
        success: bool,
        duration: float,
    ):
        """Handle mission completion from rover."""
        if task_id not in self.active_missions:
            logger.warning(f"Unknown mission completed: {task_id}")
            return

        mission = self.active_missions.pop(task_id)
        mission.completed = True
        mission.success = success
        self.completed_missions.append(mission)

        # Update rover
        if module_id in self.rovers:
            rover = self.rovers[module_id]
            rover.status = RoverStatus.IDLE
            rover.current_mission = None
            if success:
                rover.missions_completed += 1

        logger.info(
            f"Mission {task_id} {'succeeded' if success else 'failed'} "
            f"(duration={duration:.1f}s, rover={module_id})"
        )

        if self._on_mission_complete_callback:
            self._on_mission_complete_callback(mission, success)

    def get_status(self) -> dict:
        """Get auctioneer status for monitoring."""
        return {
            "rovers": {
                module_id: {
                    "status": r.status.value,
                    "payload": r.payload.value,
                    "battery_pct": r.battery_pct,
                    "missions_completed": r.missions_completed,
                    "current_mission": r.current_mission,
                }
                for module_id, r in self.rovers.items()
            },
            "queue_length": len(self.mission_queue),
            "active_missions": len(self.active_missions),
            "completed_missions": len(self.completed_missions),
            "idle_rovers": len(self.get_idle_rovers()),
        }


# Helper for generating test missions
def generate_exploration_grid(
    center: Tuple[float, float],
    spacing: float = 2.0,
    rows: int = 3,
    cols: int = 3,
) -> List[Tuple[float, float]]:
    """Generate a grid of exploration waypoints."""
    waypoints = []
    start_x = center[0] - (cols - 1) * spacing / 2
    start_y = center[1] - (rows - 1) * spacing / 2

    for row in range(rows):
        for col in range(cols):
            # Serpentine pattern
            if row % 2 == 0:
                x = start_x + col * spacing
            else:
                x = start_x + (cols - 1 - col) * spacing
            y = start_y + row * spacing
            waypoints.append((x, y))

    return waypoints
