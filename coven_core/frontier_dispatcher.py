#!/usr/bin/env python3
"""
frontier_dispatcher.py - The Cartographer on the Hill

The dock's brain for directing rovers. Like a cartographer
standing on a hill directing runners:
  "You - go north and tell me what you see"
  "You - go east and report back"

After each rover returns with data and the map is updated, this node:
1. Analyzes the current map for unexplored frontiers
2. Picks the most promising direction to explore
3. Dispatches the next available rover to that frontier
4. Repeats until exploration is complete

The rovers are simple - they just go where told and record what they see.
All the intelligence lives here on the dock.

Author: Alexander Shultis
Date: December 2025
"""

import json
import math
import numpy as np
from typing import List, Tuple, Optional, Dict
from enum import Enum
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Point


class RoverStatus(Enum):
    """Rover availability states."""
    IDLE = "idle"           # At dock, ready for mission
    DEPLOYED = "deployed"   # Out on mission
    RETURNING = "returning" # Coming back
    DOCKED = "docked"       # Just returned, transferring data


@dataclass
class Frontier:
    """An unexplored frontier region."""
    centroid: Tuple[float, float]  # World coordinates
    size: int                       # Number of frontier cells
    distance: float                 # Distance from dock
    direction: float                # Angle from dock (radians)
    score: float = 0.0              # Exploration priority


@dataclass
class RoverInfo:
    """Tracked information about a rover."""
    module_id: str
    status: RoverStatus
    last_position: Tuple[float, float]
    current_mission: Optional[str] = None
    missions_completed: int = 0
    registered_time: float = 0.0  # When the rover first registered
    dispatch_time: float = 0.0    # When last dispatched (to detect race conditions)
    previous_status: Optional[RoverStatus] = None  # Track state transitions


class FrontierDispatcher(Node):
    """
    The cartographer's brain - directs rovers to explore frontiers.

    Workflow:
    1. Start with empty/minimal map
    2. Send first rover in a default direction (e.g., north)
    3. When rover returns and SLAM processes data:
       - Receive updated map
       - Find frontier regions (edges between known and unknown)
       - Score frontiers by size, distance, direction diversity
       - Pick best frontier
       - Send next available rover
    4. Repeat until no more frontiers or coverage goal met
    """

    def __init__(self):
        super().__init__('frontier_dispatcher')

        # Parameters
        self.declare_parameter('dock_position_x', 0.0)
        self.declare_parameter('dock_position_y', 0.0)
        self.declare_parameter('min_frontier_size', 5)  # Minimum cells to consider
        self.declare_parameter('exploration_radius', 3.0)  # How far to send rovers
        self.declare_parameter('coverage_goal', 0.8)  # Stop at 80% explored
        self.declare_parameter('auto_dispatch', True)  # Auto-send rovers

        self.dock_pos = (
            self.get_parameter('dock_position_x').value,
            self.get_parameter('dock_position_y').value
        )
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.coverage_goal = self.get_parameter('coverage_goal').value
        self.auto_dispatch = self.get_parameter('auto_dispatch').value

        # State
        self.rovers: Dict[str, RoverInfo] = {}
        self.current_map: Optional[OccupancyGrid] = None
        self.frontiers: List[Frontier] = []
        self.explored_directions: List[float] = []  # Track where we've sent rovers
        self.mission_counter = 0
        self.exploration_complete = False
        self.current_coverage = 0.0

        # QoS profiles
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to map updates (from offline SLAM processor)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos
        )

        # Subscribe to mule status updates
        self.mule_status_sub = self.create_subscription(
            String, '/coven/mule_status', self._mule_status_callback, reliable_qos
        )

        # Subscribe to SLAM processor status (know when map is updated)
        self.slam_status_sub = self.create_subscription(
            String, '/coven/slam_processor_status', self._slam_status_callback, reliable_qos
        )

        # Publisher for mule missions
        self.mission_pub = self.create_publisher(
            String, '/coven/mule_mission', reliable_qos
        )

        # Publisher for dispatcher status
        self.status_pub = self.create_publisher(
            String, '/coven/dispatcher_status', reliable_qos
        )

        # Timer for initial dispatch (if no map yet, send rover in default direction)
        # Wait 30 seconds to allow all rovers to spawn, bridges to connect, and modules to start
        self.initial_timer = self.create_timer(30.0, self._initial_dispatch)
        self.initial_dispatched = False

        # Periodic dispatch check - catches idle rovers that may have been missed
        # This ensures continuous exploration even if timing between SLAM and rover status is off
        self.dispatch_timer = self.create_timer(5.0, self._periodic_dispatch_check)

        # Status broadcast timer
        self.status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info('[Dispatcher] Frontier dispatcher ready')
        self.get_logger().info('[Dispatcher] Waiting for rovers to register...')

    def _mule_status_callback(self, msg: String):
        """Track mule status updates."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        module_id = data.get('module_id')
        if not module_id:
            return

        state_str = data.get('state', 'idle')
        try:
            status = RoverStatus(state_str)
        except ValueError:
            status = RoverStatus.IDLE

        position = data.get('position', {})
        pos = (position.get('x', 0.0), position.get('y', 0.0))

        now = self.get_clock().now().nanoseconds / 1e9

        # Update or create rover entry
        if module_id not in self.rovers:
            self.rovers[module_id] = RoverInfo(
                module_id=module_id,
                status=status,
                last_position=pos,
                registered_time=now,
                previous_status=status
            )
            self.get_logger().info(f'[Dispatcher] Rover registered: {module_id}')
            # Show all registered rovers
            all_rovers = list(self.rovers.keys())
            self.get_logger().info(f'[Dispatcher] All registered rovers ({len(all_rovers)}): {all_rovers}')
        else:
            # Store previous status before updating
            prev_status = self.rovers[module_id].status
            self.rovers[module_id].previous_status = prev_status
            self.rovers[module_id].status = status
            self.rovers[module_id].last_position = pos

        # Check if rover just completed a mission
        # Only count as complete if:
        # 1. Current status is IDLE
        # 2. Previous status was NOT IDLE (actual transition happened)
        # 3. Rover has a current mission assigned
        # 4. Enough time has passed since dispatch (guard against timing glitches)
        prev = self.rovers[module_id].previous_status
        time_since_dispatch = now - self.rovers[module_id].dispatch_time
        transitioned_to_idle = (status == RoverStatus.IDLE and prev is not None and prev != RoverStatus.IDLE)

        if transitioned_to_idle and self.rovers[module_id].current_mission and time_since_dispatch > 5.0:
            self.rovers[module_id].missions_completed += 1
            mission = self.rovers[module_id].current_mission
            self.rovers[module_id].current_mission = None

            if self.exploration_complete:
                # Rover returned after exploration was marked complete
                deployed_count = sum(1 for r in self.rovers.values() if r.status == RoverStatus.DEPLOYED)
                self.get_logger().info(
                    f'[Dispatcher] Rover {module_id} returned (mission {mission}). '
                    f'Still out: {deployed_count}'
                )
                if deployed_count == 0:
                    self.get_logger().info(
                        '[Dispatcher] ════════════════════════════════════════════'
                    )
                    self.get_logger().info(
                        '[Dispatcher] ALL ROVERS RETURNED - System idle'
                    )
                    self.get_logger().info(
                        f'[Dispatcher] Final coverage: {self.current_coverage*100:.1f}%'
                    )
                    self.get_logger().info(
                        '[Dispatcher] ════════════════════════════════════════════'
                    )
            else:
                self.get_logger().info(
                    f'[Dispatcher] Rover {module_id} completed mission, '
                    f'total: {self.rovers[module_id].missions_completed}'
                )

    def _map_callback(self, msg: OccupancyGrid):
        """Receive updated map from SLAM."""
        self.current_map = msg
        # Update coverage stats whenever we get a map
        self._update_coverage()
        self._analyze_frontiers()

    def _update_coverage(self):
        """Calculate and update current map coverage."""
        if self.current_map is None:
            return

        grid = np.array(self.current_map.data)
        total = len(grid)
        known = np.sum((grid == 0) | (grid == 100))  # Free or occupied
        self.current_coverage = known / total if total > 0 else 0

    def _slam_status_callback(self, msg: String):
        """React to SLAM processor completing a map update."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get('status') == 'complete':
            self.get_logger().info('[Dispatcher] SLAM update complete, analyzing frontiers...')
            # Give time for map to be published AND rover to transition to IDLE
            # Rover takes ~2s after transfer to go IDLE, so wait 4s to be safe
            self.create_timer(4.0, self._dispatch_next_rover, callback_group=None)

    def _initial_dispatch(self):
        """Send first rover in a default direction if no map exists."""
        if self.initial_dispatched:
            return

        # Check if we have any idle rovers that have been registered for at least 5 seconds
        # This ensures the rover is fully initialized before we send it out
        now = self.get_clock().now().nanoseconds / 1e9
        idle_rovers = [
            r for r in self.rovers.values()
            if r.status == RoverStatus.IDLE and (now - r.registered_time) >= 5.0
        ]
        if not idle_rovers:
            return  # Wait for rovers to be ready

        if self.current_map is None and self.auto_dispatch:
            # No map yet - send all idle rovers in different directions to start exploring
            directions = [0.0, math.pi, math.pi/2, -math.pi/2, math.pi/4, -math.pi/4, 3*math.pi/4, -3*math.pi/4]
            self.get_logger().info(f'[Dispatcher] No map yet, sending {len(idle_rovers)} rover(s) to explore')
            for i, rover in enumerate(idle_rovers):
                direction = directions[i % len(directions)]
                dir_name = ['N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW'][i % 8]
                self.get_logger().info(f'[Dispatcher] Sending {rover.module_id} {dir_name} ({math.degrees(direction):.0f}°)')
                self._dispatch_rover(rover.module_id, direction=direction)
            self.initial_dispatched = True

    def _periodic_dispatch_check(self):
        """Periodically check for idle rovers and dispatch them.

        This catches cases where:
        - SLAM completion timing doesn't align with rover becoming idle
        - Multiple rovers return around the same time
        - Any other timing edge cases
        """
        if self.exploration_complete:
            return

        if not self.auto_dispatch:
            return

        if not self.initial_dispatched:
            return  # Wait for initial dispatch first

        # Find idle rovers that aren't currently being dispatched
        idle_rovers = [r for r in self.rovers.values() if r.status == RoverStatus.IDLE]

        if not idle_rovers:
            return

        # If we have frontiers, dispatch
        if self.frontiers:
            self.get_logger().info(
                f'[Dispatcher] Periodic check: {len(idle_rovers)} idle rover(s), '
                f'{len(self.frontiers)} frontier(s) - dispatching'
            )
            self._dispatch_next_rover()
        elif self.current_map is not None:
            # We have a map but no frontiers - re-analyze (silently)
            self._analyze_frontiers()
            if self.frontiers:
                self._dispatch_next_rover()
            # Don't log "no frontiers" on periodic checks - too spammy

    def _analyze_frontiers(self):
        """Find frontier regions in the current map."""
        if self.current_map is None:
            return

        self.frontiers = []

        grid = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width)
        )
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        # Find frontier cells (free cells adjacent to unknown)
        # -1 = unknown, 0 = free, 100 = occupied
        frontier_cells = []

        for y in range(1, grid.shape[0] - 1):
            for x in range(1, grid.shape[1] - 1):
                if grid[y, x] == 0:  # Free cell
                    # Check if adjacent to unknown
                    neighbors = [
                        grid[y-1, x], grid[y+1, x],
                        grid[y, x-1], grid[y, x+1]
                    ]
                    if -1 in neighbors:
                        # This is a frontier cell
                        world_x = origin_x + x * resolution
                        world_y = origin_y + y * resolution
                        frontier_cells.append((world_x, world_y))

        if not frontier_cells:
            self.get_logger().info('[Dispatcher] No frontiers found - exploration may be complete')
            self._check_exploration_complete()
            return

        # Cluster frontier cells into regions
        clusters = self._cluster_frontiers(frontier_cells)

        for cluster in clusters:
            if len(cluster) < self.min_frontier_size:
                continue

            # Calculate centroid
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            # Distance and direction from dock
            dx = cx - self.dock_pos[0]
            dy = cy - self.dock_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            direction = math.atan2(dy, dx)

            # Score: prefer larger frontiers, reasonable distance, new directions
            direction_novelty = self._direction_novelty(direction)
            score = len(cluster) * direction_novelty / (1.0 + distance * 0.1)

            frontier = Frontier(
                centroid=(cx, cy),
                size=len(cluster),
                distance=distance,
                direction=direction,
                score=score
            )
            self.frontiers.append(frontier)

        # Sort by score
        self.frontiers.sort(key=lambda f: f.score, reverse=True)

        self.get_logger().info(
            f'[Dispatcher] Found {len(self.frontiers)} frontier regions'
        )

    def _cluster_frontiers(self, cells: List[Tuple[float, float]],
                          threshold: float = 0.5) -> List[List[Tuple[float, float]]]:
        """Simple clustering of frontier cells by proximity."""
        if not cells:
            return []

        clusters = []
        used = [False] * len(cells)

        for i, cell in enumerate(cells):
            if used[i]:
                continue

            # Start new cluster
            cluster = [cell]
            used[i] = True

            # Find all cells within threshold
            for j, other in enumerate(cells):
                if used[j]:
                    continue
                dx = cell[0] - other[0]
                dy = cell[1] - other[1]
                if math.sqrt(dx*dx + dy*dy) < threshold:
                    cluster.append(other)
                    used[j] = True

            clusters.append(cluster)

        return clusters

    def _direction_novelty(self, direction: float) -> float:
        """Score how novel a direction is (prefer unexplored directions)."""
        if not self.explored_directions:
            return 1.0

        # Find minimum angular distance to any explored direction
        min_diff = float('inf')
        for explored in self.explored_directions:
            diff = abs(self._normalize_angle(direction - explored))
            min_diff = min(min_diff, diff)

        # Normalize: pi = completely new, 0 = same direction
        novelty = min_diff / math.pi
        return 0.5 + 0.5 * novelty  # Range: 0.5 to 1.0

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _dispatch_next_rover(self):
        """Send the next available rover to the best frontier."""
        if self.exploration_complete:
            return

        if not self.auto_dispatch:
            return

        # Find idle rover
        idle_rovers = [r for r in self.rovers.values() if r.status == RoverStatus.IDLE]
        if not idle_rovers:
            self.get_logger().info('[Dispatcher] No idle rovers available')
            return

        # Get best frontier
        if not self.frontiers:
            self.get_logger().info('[Dispatcher] No frontiers to explore')
            return

        # Show bidding info - which rovers are available
        rover_summary = []
        for r in self.rovers.values():
            status_icon = '✓' if r.status == RoverStatus.IDLE else '→' if r.status == RoverStatus.DEPLOYED else '?'
            rover_summary.append(f'{r.module_id}[{status_icon}]')

        self.get_logger().info(
            f'[Dispatcher] ─── Rover Selection ───'
        )
        self.get_logger().info(
            f'[Dispatcher] Available rovers: {", ".join(rover_summary)}'
        )
        self.get_logger().info(
            f'[Dispatcher] Idle candidates: {[r.module_id for r in idle_rovers]}'
        )

        best = self.frontiers[0]
        rover = idle_rovers[0]

        self.get_logger().info(
            f'[Dispatcher] Selected: {rover.module_id} → frontier at '
            f'({best.centroid[0]:.1f}, {best.centroid[1]:.1f}), '
            f'direction: {math.degrees(best.direction):.0f}°'
        )

        self._dispatch_rover(rover.module_id, target=best.centroid, direction=best.direction)

    def _dispatch_rover(self, module_id: str,
                       target: Optional[Tuple[float, float]] = None,
                       direction: Optional[float] = None):
        """Send a rover on an exploration mission."""
        self.mission_counter += 1
        mission_id = f'frontier_{self.mission_counter:03d}'

        # If no target, use direction + exploration_radius
        if target is None and direction is not None:
            target = (
                self.dock_pos[0] + self.exploration_radius * math.cos(direction),
                self.dock_pos[1] + self.exploration_radius * math.sin(direction)
            )

        if target is None:
            self.get_logger().error('[Dispatcher] No target specified')
            return

        # Track explored direction
        if direction is not None:
            self.explored_directions.append(direction)

        # Build mission command
        # Format: START:<mission_id>:<waypoints>
        waypoints = f'{target[0]:.2f},{target[1]:.2f}'
        cmd = f'START:{mission_id}:{waypoints}'

        msg = String()
        msg.data = cmd
        self.mission_pub.publish(msg)

        # Update rover state
        if module_id in self.rovers:
            self.rovers[module_id].status = RoverStatus.DEPLOYED
            self.rovers[module_id].current_mission = mission_id
            self.rovers[module_id].dispatch_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f'[Dispatcher] Mission {mission_id} dispatched to {module_id}: {waypoints}'
        )

    def _check_exploration_complete(self):
        """Check if we've achieved our exploration goal."""
        if self.current_map is None:
            return

        # Coverage already updated by _update_coverage()
        self.get_logger().info(f'[Dispatcher] Map coverage: {self.current_coverage*100:.1f}%')

        if self.current_coverage >= self.coverage_goal and not self.exploration_complete:
            self.exploration_complete = True
            deployed_count = sum(1 for r in self.rovers.values() if r.status == RoverStatus.DEPLOYED)
            self.get_logger().info(
                '[Dispatcher] ════════════════════════════════════════════'
            )
            self.get_logger().info(
                '[Dispatcher] EXPLORATION COMPLETE!'
            )
            self.get_logger().info(
                f'[Dispatcher] Coverage: {self.current_coverage*100:.1f}% (goal: {self.coverage_goal*100:.1f}%)'
            )
            self.get_logger().info(
                f'[Dispatcher] Missions dispatched: {self.mission_counter}'
            )
            if deployed_count > 0:
                self.get_logger().info(
                    f'[Dispatcher] Waiting for {deployed_count} rover(s) to return...'
                )
            self.get_logger().info(
                '[Dispatcher] ════════════════════════════════════════════'
            )

    def _publish_status(self):
        """Broadcast dispatcher status."""
        idle_count = sum(1 for r in self.rovers.values() if r.status == RoverStatus.IDLE)
        deployed_count = sum(1 for r in self.rovers.values() if r.status == RoverStatus.DEPLOYED)

        status = {
            'exploration_complete': self.exploration_complete,
            'coverage_pct': round(self.current_coverage * 100, 1),
            'coverage_goal_pct': round(self.coverage_goal * 100, 1),
            'frontier_count': len(self.frontiers),
            'rovers_registered': len(self.rovers),
            'rovers_idle': idle_count,
            'rovers_deployed': deployed_count,
            'missions_dispatched': self.mission_counter,
        }

        if self.frontiers:
            best = self.frontiers[0]
            status['best_frontier'] = {
                'x': best.centroid[0],
                'y': best.centroid[1],
                'size': best.size,
                'direction_deg': math.degrees(best.direction)
            }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDispatcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
