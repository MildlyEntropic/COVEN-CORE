"""
exploration.py — COVEN Phase 1 Navigation Integration

Autonomous exploration behavior for COVEN modules using frontier-based
exploration with Nav2 and SLAM integration.

Responsibilities:
- Generate exploration goals based on frontiers (known/unknown boundaries)
- Navigate to waypoints using Nav2 BasicNavigator
- Monitor map building progress via SLAM
- Determine exploration completion
- Return to dock coordinates

Author: Alexander Shultis
Date: October 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
# --- Standard library ---
import math
import time
from typing import List, Tuple, Optional

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- NumPy for frontier detection ---
try:
    import numpy as np
except ImportError:
    np = None


# ------------------------
# --- Constants ---
# ------------------------
FRONTIER_SEARCH_RADIUS = 3.0  # meters - how far to look for frontiers
MIN_FRONTIER_SIZE = 10  # minimum cells to be considered a frontier
EXPLORATION_TIMEOUT = 300.0  # seconds - max exploration time
COVERAGE_THRESHOLD = 0.80  # 80% coverage considered complete
NO_FRONTIER_LIMIT = 3  # iterations without frontiers before stopping


# ------------------------
# --- Explorer Class ---
# ------------------------
class Explorer:
    """
    Autonomous exploration manager using frontier-based navigation.
    """

    def __init__(self, node: Node, navigator: BasicNavigator):
        """
        Initialize the explorer.

        Args:
            node: ROS2 node for logging and subscriptions
            navigator: Nav2 BasicNavigator instance
        """
        self.node = node
        self.navigator = navigator
        self.current_map: Optional[OccupancyGrid] = None
        self.start_time = None
        self.start_pose: Optional[PoseStamped] = None
        self.explored_cells = 0
        self.total_reachable_cells = 0
        self.no_frontier_count = 0

        # Subscribe to map updates from SLAM
        self.map_sub = node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

        self.node.get_logger().info("Explorer initialized")

    def _map_callback(self, msg: OccupancyGrid):
        """Update current map from SLAM."""
        self.current_map = msg
        # Count explored cells (free or occupied, not unknown)
        if np:
            data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.explored_cells = np.sum((data >= 0) & (data <= 100))
            # Estimate total reachable (avoid counting walls as unexplored)
            self.total_reachable_cells = max(self.explored_cells,
                                             msg.info.width * msg.info.height // 2)

    def get_current_pose(self) -> PoseStamped:
        """Get robot's current pose from navigator."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # Try to get current pose from navigator (may require TF lookup in full impl)
        # For now, return a default - in production, use TF2 to get actual pose
        return pose

    def find_frontiers(self) -> List[PoseStamped]:
        """
        Detect frontiers in the current map (boundaries between known/unknown).

        Returns:
            List of PoseStamped goals at frontier locations
        """
        if not self.current_map or not np:
            self.node.get_logger().warn("No map available or NumPy not installed")
            return []

        goals = []
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        # Convert map data to numpy array
        data = np.array(self.current_map.data).reshape((height, width))

        # Find frontier cells: free space (-1 = unknown, 0 = free, 100 = occupied)
        # Frontier = free cell adjacent to unknown cell
        free_cells = (data == 0)
        unknown_cells = (data == -1)

        # Use convolution to find adjacency (simple 4-neighbor check)
        from scipy.ndimage import binary_dilation
        if not binary_dilation:
            self.node.get_logger().warn("scipy not available for frontier detection")
            return self._generate_random_goals()

        # Dilate unknown cells by 1 to find cells adjacent to unknown
        unknown_dilated = binary_dilation(unknown_cells, structure=np.ones((3, 3)))

        # Frontier cells are free AND adjacent to unknown
        frontier_cells = free_cells & unknown_dilated

        # Find clusters of frontier cells
        frontier_indices = np.argwhere(frontier_cells)

        if len(frontier_indices) < MIN_FRONTIER_SIZE:
            self.node.get_logger().info(f"No significant frontiers found ({len(frontier_indices)} cells)")
            self.no_frontier_count += 1
            return []

        self.no_frontier_count = 0  # Reset counter when frontiers found

        # Cluster frontiers and select representative points
        # Simple approach: sample every Nth frontier cell
        sample_step = max(1, len(frontier_indices) // 5)  # Up to 5 goals

        for i in range(0, len(frontier_indices), sample_step):
            y, x = frontier_indices[i]

            # Convert grid coordinates to world coordinates
            world_x = origin_x + (x + 0.5) * resolution
            world_y = origin_y + (y + 0.5) * resolution

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            goal.pose.position.x = float(world_x)
            goal.pose.position.y = float(world_y)
            goal.pose.orientation.w = 1.0  # Face forward

            goals.append(goal)

            if len(goals) >= 5:  # Limit to 5 goals at a time
                break

        self.node.get_logger().info(f"Found {len(goals)} frontier goals from {len(frontier_indices)} frontier cells")
        return goals

    def _generate_random_goals(self) -> List[PoseStamped]:
        """
        Generate random exploration goals (fallback when frontier detection unavailable).

        Returns:
            List of random goal poses within exploration radius
        """
        goals = []
        if not self.start_pose:
            return goals

        # Generate 3 random goals in a circle around start
        for angle in [0, 120, 240]:
            rad = math.radians(angle)
            distance = FRONTIER_SEARCH_RADIUS

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            goal.pose.position.x = self.start_pose.pose.position.x + distance * math.cos(rad)
            goal.pose.position.y = self.start_pose.pose.position.y + distance * math.sin(rad)
            goal.pose.orientation.w = 1.0

            goals.append(goal)

        self.node.get_logger().info(f"Generated {len(goals)} random exploration goals (fallback mode)")
        return goals

    def explore(self, duration: float = EXPLORATION_TIMEOUT) -> Tuple[bool, dict]:
        """
        Execute exploration mission.

        Args:
            duration: Maximum exploration time in seconds

        Returns:
            Tuple of (success, metrics_dict)
        """
        self.start_time = time.time()
        self.start_pose = self.get_current_pose()

        self.node.get_logger().info(f"Starting exploration (max duration: {duration}s)")

        iteration = 0
        total_distance = 0.0

        while True:
            iteration += 1
            elapsed = time.time() - self.start_time

            # Check termination conditions
            if elapsed >= duration:
                self.node.get_logger().info("Exploration timeout reached")
                break

            if self.no_frontier_count >= NO_FRONTIER_LIMIT:
                self.node.get_logger().info("No new frontiers found - exploration complete")
                break

            coverage = self._calculate_coverage()
            if coverage >= COVERAGE_THRESHOLD:
                self.node.get_logger().info(f"Coverage threshold reached: {coverage:.1%}")
                break

            # Find and navigate to frontiers
            frontiers = self.find_frontiers()

            if not frontiers:
                # Try random exploration if no frontiers
                frontiers = self._generate_random_goals()

            if not frontiers:
                self.node.get_logger().warn("No exploration goals available")
                break

            # Navigate to closest frontier
            closest_goal = self._select_closest_goal(frontiers)
            if not closest_goal:
                break

            self.node.get_logger().info(
                f"Iteration {iteration}: Navigating to frontier at "
                f"({closest_goal.pose.position.x:.2f}, {closest_goal.pose.position.y:.2f})"
            )

            # Send navigation goal
            self.navigator.goToPose(closest_goal)

            # Wait for navigation to complete
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.node, timeout_sec=0.1)

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.node.get_logger().info("Reached frontier waypoint")
                total_distance += self._estimate_distance(closest_goal)
            elif result == TaskResult.CANCELED:
                self.node.get_logger().warn("Navigation canceled")
                break
            elif result == TaskResult.FAILED:
                self.node.get_logger().warn("Navigation failed, trying next frontier")
                continue

        # Compile metrics
        final_coverage = self._calculate_coverage()
        metrics = {
            "duration": time.time() - self.start_time,
            "iterations": iteration,
            "coverage": final_coverage,
            "explored_cells": self.explored_cells,
            "distance_traveled": total_distance
        }

        self.node.get_logger().info(
            f"Exploration complete: {iteration} iterations, "
            f"{final_coverage:.1%} coverage, "
            f"{metrics['duration']:.1f}s elapsed"
        )

        return True, metrics

    def return_to_dock(self, dock_pose: PoseStamped) -> bool:
        """
        Navigate back to the docking station.

        Args:
            dock_pose: Pose of the dock

        Returns:
            True if successfully returned, False otherwise
        """
        self.node.get_logger().info(
            f"Returning to dock at ({dock_pose.pose.position.x:.2f}, "
            f"{dock_pose.pose.position.y:.2f})"
        )

        self.navigator.goToPose(dock_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.node.get_logger().info("Successfully returned to dock")
            return True
        else:
            self.node.get_logger().error(f"Failed to return to dock: {result}")
            return False

    def _calculate_coverage(self) -> float:
        """Calculate current map coverage percentage."""
        if self.total_reachable_cells == 0:
            return 0.0
        return self.explored_cells / self.total_reachable_cells

    def _select_closest_goal(self, goals: List[PoseStamped]) -> Optional[PoseStamped]:
        """Select the closest goal from a list."""
        if not goals:
            return None

        # For simplicity, return first goal
        # In production, calculate actual distance to current pose
        return goals[0]

    def _estimate_distance(self, goal: PoseStamped) -> float:
        """Estimate distance traveled to goal."""
        if not self.start_pose:
            return 0.0

        dx = goal.pose.position.x - self.start_pose.pose.position.x
        dy = goal.pose.position.y - self.start_pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
