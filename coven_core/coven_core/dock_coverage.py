"""
dock_coverage.py — Multi-rover coverage mission coordinator.

Manages sector division, dispatch, recharge cycling, sector splitting,
and mission completion for coverage exploration missions.
Instantiated by the Dock node and accesses ROS2 via self._node.

Author: Alexander Shultis
Date: December 2025
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

from std_msgs.msg import String

import coven_core.common as common
from coven_core.common import (
    COLOR_GREEN, COLOR_YELLOW, COLOR_RESET,
    CoverageGoal, CoverageMissionComplete, MissionRequest, Sector,
)

if TYPE_CHECKING:
    from coven_core.dock_node import Dock


class CoverageCoordinator:
    """Handles multi-rover coverage exploration missions."""

    def __init__(self, node: 'Dock'):
        self._node = node

        # Coverage exploration state
        self.mission_active = False
        self.target = 0.95
        self.goal: CoverageGoal | None = None
        self.global_coverage = 0.0
        self.rover_coverage: dict[str, float] = {}
        self.rover_maps: dict[str, str] = {}
        self.sectors: list[Sector] = []
        self.dispatch_cycles = 0
        self.recharge_timers: dict = {}
        self._lock = threading.Lock()

    def on_coverage_status(self, msg):
        """Handle periodic coverage status updates from rovers."""
        status = common.coverage_status_decode(msg)
        if not status:
            return

        with self._lock:
            self.rover_coverage[status.module_id] = status.current_coverage

            self._node.get_logger().info(
                f"Coverage status from {status.module_id}: "
                f"{status.current_coverage:.1%} coverage, "
                f"{status.battery_remaining:.0%} battery, "
                f"{status.frontiers_remaining} frontiers"
                + (f" (returning: {status.reason})" if status.returning_to_dock else "")
            )

    def start_mission(self, goal: CoverageGoal):
        """Start a multi-rover coverage exploration mission.

        Divides the arena into sectors and dispatches rovers to explore.
        """
        self._node.get_logger().info(
            f"{COLOR_GREEN}Starting coverage mission: "
            f"target={goal.target_coverage:.0%}, max_time={goal.max_exploration_time}s{COLOR_RESET}"
        )

        with self._lock:
            self.mission_active = True
            self.target = goal.target_coverage
            self.goal = goal
            self.global_coverage = 0.0
            self.rover_coverage = {}
            self.rover_maps = {}
            self.dispatch_cycles = 0

        available_rovers = self._get_available_rovers()
        if not available_rovers:
            self._node.get_logger().warn("No available rovers for coverage mission")
            return

        self.sectors = self._divide_into_sectors(len(available_rovers), goal.sector_bounds)

        for rover_id, sector in zip(available_rovers, self.sectors):
            sector.assigned_to = rover_id
            self._dispatch_coverage_task(rover_id, sector, goal)

        self.dispatch_cycles = 1

    def _get_available_rovers(self) -> list:
        """Get list of module IDs that are available for tasks."""
        available = []
        with self._node._mod_lock:
            for mod_id, mod in self._node.modules.items():
                if mod["state"] == common.DockState.ENABLED and not mod["paused"]:
                    available.append(mod_id)
        return available

    def _divide_into_sectors(self, num_rovers: int, bounds: tuple = None) -> list:
        """Divide the exploration area into sectors for rovers.

        Uses simple quadrant division for 2-4 rovers.
        For more rovers, uses grid-based division.
        """
        if bounds:
            x_min, y_min, x_max, y_max = bounds
        else:
            x_min, y_min, x_max, y_max = -7.5, -7.5, 7.5, 7.5

        width = x_max - x_min
        height = y_max - y_min
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2

        sectors = []

        if num_rovers == 1:
            sectors.append(Sector(name="ALL", bounds=(x_min, y_min, x_max, y_max)))
        elif num_rovers == 2:
            sectors.append(Sector(name="WEST", bounds=(x_min, y_min, cx, y_max)))
            sectors.append(Sector(name="EAST", bounds=(cx, y_min, x_max, y_max)))
        elif num_rovers <= 4:
            quadrants = [
                ("NE", (cx, cy, x_max, y_max)),
                ("NW", (x_min, cy, cx, y_max)),
                ("SW", (x_min, y_min, cx, cy)),
                ("SE", (cx, y_min, x_max, cy)),
            ]
            for i, (name, b) in enumerate(quadrants[:num_rovers]):
                sectors.append(Sector(name=name, bounds=b))
        else:
            cols = int(num_rovers ** 0.5)
            rows = (num_rovers + cols - 1) // cols
            cell_w = width / cols
            cell_h = height / rows

            idx = 0
            for row in range(rows):
                for col in range(cols):
                    if idx >= num_rovers:
                        break
                    sx = x_min + col * cell_w
                    sy = y_min + row * cell_h
                    sectors.append(Sector(
                        name=f"G{row}_{col}",
                        bounds=(sx, sy, sx + cell_w, sy + cell_h)
                    ))
                    idx += 1

        self._node.get_logger().info(
            f"Divided arena into {len(sectors)} sectors: "
            f"{[s.name for s in sectors]}"
        )
        return sectors

    def _dispatch_coverage_task(self, rover_id: str, sector: Sector, goal: CoverageGoal):
        """Dispatch a coverage exploration task to a specific rover."""
        sector_goal = CoverageGoal(
            target_coverage=goal.target_coverage,
            sector=sector.name,
            sector_bounds=sector.bounds,
            max_exploration_time=goal.max_exploration_time,
            return_on_low_battery=goal.return_on_low_battery,
            battery_return_threshold=goal.battery_return_threshold,
        )

        mission = MissionRequest(
            task="coverage",
            coverage_goal=sector_goal,
            return_to_dock=True
        )

        task_data = common.mission_req_encode(mission)

        self._node.get_logger().info(
            f"{COLOR_YELLOW}Dispatching {rover_id} to sector {sector.name} "
            f"(bounds: {sector.bounds}){COLOR_RESET}"
        )

        with self._node._mod_lock:
            mod = self._node.modules.get(rover_id)
            if mod:
                mod["paused"] = True

        task_req = common.TaskReq(module_id=rover_id, task=task_data)
        self._node.pub_task_req.publish(String(data=common.task_req_encode(task_req)))

    def handle_task_complete(self, tc: common.TaskComplete):
        """Handle task completion during a coverage mission.

        Merges map data and checks if global coverage target is met.
        Re-dispatches rover if more exploration is needed.
        """
        with self._lock:
            if not self.mission_active:
                return

            rover_coverage = tc.exploration_metrics.get("coverage", 0.0)
            self.rover_coverage[tc.module_id] = rover_coverage

            if tc.map_data:
                self.rover_maps[tc.module_id] = tc.map_data

            self._node.get_logger().info(
                f"{COLOR_GREEN}Rover {tc.module_id} returned. "
                f"Global coverage from SLAM: {self.global_coverage:.1%} "
                f"(target: {self.target:.1%}){COLOR_RESET}"
            )

            if self.global_coverage >= self.target:
                self._complete_mission(success=True)
                return

            return_reason = tc.exploration_metrics.get("return_reason", "")

            if return_reason == "low_battery":
                rover_id = tc.module_id
                self._node.get_logger().info(
                    f"{rover_id} returned for battery recharge - "
                    f"re-dispatch in {self._node.recharge_delay:.0f}s"
                )
                if rover_id in self.recharge_timers:
                    self.recharge_timers[rover_id].cancel()
                self.recharge_timers[rover_id] = self._node.create_timer(
                    self._node.recharge_delay,
                    lambda rid=rover_id: self._recharge_complete(rid),
                )

            elif return_reason == "no_frontiers":
                self._node.get_logger().info(
                    f"{tc.module_id} completed sector - no more frontiers"
                )
                self._reassign_to_unexplored_sector(tc.module_id)

            else:
                self._redispatch_rover(tc.module_id)

    def _redispatch_rover(self, rover_id: str):
        """Re-dispatch a rover to continue coverage exploration."""
        with self._lock:
            if not self.mission_active or not self.goal:
                return

            sector = None
            for s in self.sectors:
                if s.assigned_to == rover_id:
                    sector = s
                    break

            if not sector:
                self._node.get_logger().warn(f"No sector assigned to {rover_id}")
                return

            self.dispatch_cycles += 1

            self._node.get_logger().info(
                f"{COLOR_YELLOW}Re-dispatching {rover_id} to sector {sector.name} "
                f"(dispatch cycle {self.dispatch_cycles}){COLOR_RESET}"
            )

            self._dispatch_coverage_task(rover_id, sector, self.goal)

    def _recharge_complete(self, rover_id: str):
        """Called after recharge delay expires — cancel the timer and re-dispatch."""
        if rover_id in self.recharge_timers:
            self.recharge_timers[rover_id].cancel()
            del self.recharge_timers[rover_id]
        self._node.get_logger().info(
            f"{COLOR_GREEN}{rover_id} recharge complete - re-dispatching{COLOR_RESET}"
        )
        self._redispatch_rover(rover_id)

    def _split_sector(self, sector: Sector, rover_id: str) -> Sector:
        """Split a sector along its longer axis, returning the new half for rover_id."""
        x_min, y_min, x_max, y_max = sector.bounds
        width = x_max - x_min
        height = y_max - y_min

        if width >= height:
            x_mid = x_min + width / 2
            sector.bounds = (x_min, y_min, x_mid, y_max)
            new_bounds = (x_mid, y_min, x_max, y_max)
            split_axis = "X"
        else:
            y_mid = y_min + height / 2
            sector.bounds = (x_min, y_min, x_max, y_mid)
            new_bounds = (x_min, y_mid, x_max, y_max)
            split_axis = "Y"

        new_sector = Sector(
            name=f"{sector.name}_{rover_id[:8]}",
            bounds=new_bounds,
            assigned_to=rover_id,
        )
        self.sectors.append(new_sector)

        self._node.get_logger().info(
            f"Split sector {sector.name} along {split_axis}-axis: "
            f"{sector.assigned_to} keeps {sector.bounds}, "
            f"{rover_id} gets {new_bounds}"
        )
        return new_sector

    def _reassign_to_unexplored_sector(self, rover_id: str):
        """Reassign a rover to help with an unexplored sector."""
        with self._lock:
            if not self.mission_active:
                return

            least_covered = None
            min_coverage = 1.0

            for sector in self.sectors:
                if sector.assigned_to and sector.assigned_to != rover_id:
                    other_coverage = self.rover_coverage.get(sector.assigned_to, 0.0)
                    if other_coverage < min_coverage:
                        min_coverage = other_coverage
                        least_covered = sector

            if least_covered and min_coverage < self.target:
                new_sector = self._split_sector(least_covered, rover_id)
                if self.goal:
                    self._dispatch_coverage_task(rover_id, new_sector, self.goal)
            else:
                self._node.get_logger().info(
                    f"{rover_id} completed exploration - all sectors covered"
                )

    def _complete_mission(self, success: bool = True):
        """Complete the coverage mission and log final statistics."""
        with self._lock:
            self.mission_active = False

            mission_complete = CoverageMissionComplete(
                success=success,
                total_coverage=self.global_coverage,
                target_coverage=self.target,
                rovers_dispatched=len(self.rover_coverage),
                dispatch_cycles=self.dispatch_cycles,
            )

            result_str = "SUCCESS" if success else "INCOMPLETE"
            self._node.get_logger().info(
                f"{COLOR_GREEN}{'='*50}\n"
                f"COVERAGE MISSION {result_str}\n"
                f"  Target: {self.target:.0%}\n"
                f"  Achieved: {self.global_coverage:.1%}\n"
                f"  Rovers: {len(self.rover_coverage)}\n"
                f"  Dispatch cycles: {self.dispatch_cycles}\n"
                f"{'='*50}{COLOR_RESET}"
            )
