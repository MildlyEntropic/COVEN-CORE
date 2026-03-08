#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
frontier_dispatcher.py - Frontier Detection and Rover Dispatch

Centralized exploration coordinator for the COVEN dock. This node:
1. Analyzes the current map for unexplored frontiers
2. Scores frontiers by size, distance, and information gain
3. Dispatches available rovers to promising frontiers
4. Repeats until exploration is complete

Rovers collect sensor data and return it to the dock. All SLAM processing
and exploration planning occurs here where power and cooling are available.

Pure analysis lives in frontier_analysis.py; data types in dispatch_tracker.py.

Author: Alexander Shultis
Date: December 2025
"""

import json
import math
import numpy as np
from typing import List, Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from coven_core.frontier_analysis import Frontier, analyze_frontiers
from coven_core.dispatch_tracker import DispatchStatus, RoverInfo


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

        # Subscribe to rover status updates
        self.rover_status_sub = self.create_subscription(
            String, '/coven/rover_status', self._rover_status_callback, reliable_qos
        )

        # Subscribe to SLAM processor status (know when map is updated)
        self.slam_status_sub = self.create_subscription(
            String, '/coven/slam_processor_status', self._slam_status_callback, reliable_qos
        )

        # Subscribe to auctioneer status (track rover availability)
        self.auctioneer_status_sub = self.create_subscription(
            String, '/coven/auctioneer_status', self._auctioneer_status_callback, reliable_qos
        )

        # Publisher for missions (to auctioneer)
        self.mission_pub = self.create_publisher(
            String, '/coven/mission_request', reliable_qos
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

        # Auctioneer-tracked state (from /coven/auctioneer_status)
        self.auctioneer_idle_count = 0
        self.auctioneer_queue_size = 0

        self.get_logger().info('[Dispatcher] Frontier dispatcher ready')
        self.get_logger().info('[Dispatcher] Waiting for rovers to register...')

    def _rover_status_callback(self, msg: String):
        """Track rover status updates."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        module_id = data.get('module_id')
        if not module_id:
            return

        state_str = data.get('state', 'idle')
        try:
            status = DispatchStatus(state_str)
        except ValueError:
            status = DispatchStatus.IDLE

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
            all_rovers = list(self.rovers.keys())
            self.get_logger().info(f'[Dispatcher] All registered rovers ({len(all_rovers)}): {all_rovers}')
        else:
            prev_status = self.rovers[module_id].status
            self.rovers[module_id].previous_status = prev_status
            self.rovers[module_id].status = status
            self.rovers[module_id].last_position = pos

        # Check if rover just completed a mission
        prev = self.rovers[module_id].previous_status
        time_since_dispatch = now - self.rovers[module_id].dispatch_time
        transitioned_to_idle = (status == DispatchStatus.IDLE and prev is not None and prev != DispatchStatus.IDLE)

        if transitioned_to_idle and self.rovers[module_id].current_mission and time_since_dispatch > 5.0:
            self.rovers[module_id].missions_completed += 1
            mission = self.rovers[module_id].current_mission
            self.rovers[module_id].current_mission = None

            if self.exploration_complete:
                deployed_count = sum(1 for r in self.rovers.values() if r.status == DispatchStatus.DEPLOYED)
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
        self._update_coverage()
        self._refresh_frontiers()

    def _update_coverage(self):
        """Calculate and update current map coverage."""
        if self.current_map is None:
            return

        grid = np.array(self.current_map.data)
        total = len(grid)
        known = np.sum(grid >= 0)  # Any non-unknown cell (free=0, occupied=1-100)
        self.current_coverage = known / total if total > 0 else 0

    def _refresh_frontiers(self):
        """Re-analyze frontiers from the current map."""
        if self.current_map is None:
            return

        self.frontiers = analyze_frontiers(
            grid_data=np.array(self.current_map.data),
            width=self.current_map.info.width,
            height=self.current_map.info.height,
            resolution=self.current_map.info.resolution,
            origin_x=self.current_map.info.origin.position.x,
            origin_y=self.current_map.info.origin.position.y,
            dock_pos=self.dock_pos,
            explored_directions=self.explored_directions,
            min_frontier_size=self.min_frontier_size,
        )

        if not self.frontiers:
            self.get_logger().info('[Dispatcher] No frontiers found - exploration may be complete')
            self._check_exploration_complete()
        else:
            self.get_logger().info(
                f'[Dispatcher] Found {len(self.frontiers)} frontier regions'
            )

    def _slam_status_callback(self, msg: String):
        """React to SLAM processor completing a map update."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get('status') == 'complete':
            self.get_logger().info('[Dispatcher] SLAM update complete, analyzing frontiers...')
            # Give time for map to be published AND rover to transition to IDLE
            # Destroy any previous one-shot timer to avoid accumulation
            if hasattr(self, '_slam_dispatch_timer') and self._slam_dispatch_timer is not None:
                self.destroy_timer(self._slam_dispatch_timer)
            self._slam_dispatch_timer = self.create_timer(
                4.0, self._slam_dispatch_once
            )

    def _auctioneer_status_callback(self, msg: String):
        """Track rover availability from the auctioneer."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        rovers_data = data.get('rovers', {})
        for module_id, rover_info in rovers_data.items():
            if module_id not in self.rovers:
                self.rovers[module_id] = RoverInfo(
                    module_id=module_id,
                    status=DispatchStatus.IDLE,
                    last_position=(rover_info.get('x', 0.0), rover_info.get('y', 0.0)),
                    registered_time=self.get_clock().now().nanoseconds / 1e9
                )
            else:
                self.rovers[module_id].last_position = (
                    rover_info.get('x', 0.0),
                    rover_info.get('y', 0.0)
                )

    def _initial_dispatch(self):
        """Send first rover in a default direction if no map exists."""
        if self.initial_dispatched:
            self.destroy_timer(self.initial_timer)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        idle_rovers = [
            r for r in self.rovers.values()
            if r.status == DispatchStatus.IDLE and (now - r.registered_time) >= 5.0
        ]
        if not idle_rovers:
            return

        if self.current_map is None and self.auto_dispatch:
            directions = [0.0, math.pi, math.pi/2, -math.pi/2, math.pi/4, -math.pi/4, 3*math.pi/4, -3*math.pi/4]
            self.get_logger().info(f'[Dispatcher] No map yet, sending {len(idle_rovers)} rover(s) to explore')
            for i, rover in enumerate(idle_rovers):
                direction = directions[i % len(directions)]
                dir_name = ['N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW'][i % 8]
                self.get_logger().info(f'[Dispatcher] Sending {rover.module_id} {dir_name} ({math.degrees(direction):.0f}°)')
                self._dispatch_rover(rover.module_id, direction=direction)
            self.initial_dispatched = True
            self.destroy_timer(self.initial_timer)

    def _periodic_dispatch_check(self):
        """Periodically check for idle rovers and dispatch them."""
        if self.exploration_complete or not self.auto_dispatch or not self.initial_dispatched:
            return

        idle_rovers = [r for r in self.rovers.values() if r.status == DispatchStatus.IDLE]
        if not idle_rovers:
            return

        if self.frontiers:
            self.get_logger().info(
                f'[Dispatcher] Periodic check: {len(idle_rovers)} idle rover(s), '
                f'{len(self.frontiers)} frontier(s) - dispatching all'
            )
            self._dispatch_all_idle_rovers()
        elif self.current_map is not None:
            self._refresh_frontiers()
            if self.frontiers:
                self._dispatch_all_idle_rovers()

    def _dispatch_next_rover(self):
        """Send the next available rover to the best frontier.

        Legacy single-dispatch method - use _dispatch_all_idle_rovers for parallel dispatch.
        """
        self._dispatch_all_idle_rovers(max_dispatches=1)

    def _slam_dispatch_once(self):
        """One-shot callback after SLAM completion — dispatch then destroy timer."""
        self._dispatch_all_idle_rovers()
        if hasattr(self, '_slam_dispatch_timer') and self._slam_dispatch_timer is not None:
            self.destroy_timer(self._slam_dispatch_timer)
            self._slam_dispatch_timer = None

    def _dispatch_all_idle_rovers(self, max_dispatches: int = 0):
        """Send ALL available idle rovers to frontiers in parallel.

        Args:
            max_dispatches: Maximum rovers to dispatch (0 = unlimited)
        """
        if self.exploration_complete or not self.auto_dispatch:
            return

        idle_rovers = [r for r in self.rovers.values() if r.status == DispatchStatus.IDLE]
        if not idle_rovers:
            self.get_logger().info('[Dispatcher] No idle rovers available')
            return

        # Refresh frontier analysis to get current state
        if self.current_map is not None:
            self._refresh_frontiers()

        if not self.frontiers:
            self.get_logger().info('[Dispatcher] No frontiers to explore')
            return

        # Show bidding info
        rover_summary = []
        for r in self.rovers.values():
            status_icon = '✓' if r.status == DispatchStatus.IDLE else '→' if r.status == DispatchStatus.DEPLOYED else '?'
            rover_summary.append(f'{r.module_id}[{status_icon}]')

        self.get_logger().info('[Dispatcher] ─── Parallel Dispatch ───')
        self.get_logger().info(f'[Dispatcher] Rovers: {", ".join(rover_summary)}')
        self.get_logger().info(f'[Dispatcher] Idle: {len(idle_rovers)}, Frontiers: {len(self.frontiers)}')

        # Dispatch each idle rover to a different frontier
        dispatched = 0
        frontier_idx = 0

        for rover in idle_rovers:
            if max_dispatches > 0 and dispatched >= max_dispatches:
                break
            if frontier_idx >= len(self.frontiers):
                self.get_logger().info(
                    f'[Dispatcher] Ran out of frontiers ({len(self.frontiers)}) '
                    f'for {len(idle_rovers) - dispatched} remaining rovers'
                )
                break

            frontier = self.frontiers[frontier_idx]

            self.get_logger().info(
                f'[Dispatcher] Dispatching: {rover.module_id} → '
                f'frontier #{frontier_idx + 1} at ({frontier.centroid[0]:.1f}, {frontier.centroid[1]:.1f}), '
                f'{math.degrees(frontier.direction):.0f}°'
            )

            self._dispatch_rover(rover.module_id, target=frontier.centroid, direction=frontier.direction)

            dispatched += 1
            frontier_idx += 1

        if dispatched > 0:
            self.get_logger().info(f'[Dispatcher] ─── Dispatched {dispatched} rover(s) ───')

    def _dispatch_rover(self, module_id: str,
                       target=None,
                       direction=None):
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

        # Build mission request for auctioneer
        mission_request = {
            "task_type": "explore",
            "waypoints": [{"x": target[0], "y": target[1]}],
            "priority": 0,
        }

        msg = String()
        msg.data = json.dumps(mission_request)
        self.mission_pub.publish(msg)

        # Mark rover as DEPLOYED to prevent double-dispatch
        if module_id in self.rovers:
            self.rovers[module_id].status = DispatchStatus.DEPLOYED
            self.rovers[module_id].current_mission = mission_id
            self.rovers[module_id].dispatch_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f'[Dispatcher] Mission {mission_id} queued for {module_id}: {target}'
        )

    def _check_exploration_complete(self):
        """Check if we've achieved our exploration goal."""
        if self.current_map is None:
            return

        self.get_logger().info(f'[Dispatcher] Map coverage: {self.current_coverage*100:.1f}%')

        if self.current_coverage >= self.coverage_goal and not self.exploration_complete:
            self.exploration_complete = True
            deployed_count = sum(1 for r in self.rovers.values() if r.status == DispatchStatus.DEPLOYED)
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
        idle_count = sum(1 for r in self.rovers.values() if r.status == DispatchStatus.IDLE)
        deployed_count = sum(1 for r in self.rovers.values() if r.status == DispatchStatus.DEPLOYED)

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
