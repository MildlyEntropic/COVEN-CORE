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
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- TF2 for pose lookups ---
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException  # Base class for all TF2 exceptions

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

    def __init__(self, node: Node, navigator: BasicNavigator, robot_namespace: str = ""):
        """
        Initialize the explorer.

        Args:
            node: ROS2 node for logging and subscriptions
            navigator: Nav2 BasicNavigator instance
            robot_namespace: Robot namespace for TF frames (e.g., "RR-abc123")
        """
        self.node = node
        self.navigator = navigator
        self.robot_namespace = robot_namespace
        self.current_map: Optional[OccupancyGrid] = None
        self.start_time = None
        self.start_pose: Optional[PoseStamped] = None
        self.last_pose: Optional[PoseStamped] = None  # Track for distance calculation
        self.explored_cells = 0
        self.total_reachable_cells = 0
        self.no_frontier_count = 0

        # TF2 buffer and listener for pose lookups
        # IMPORTANT: Use the navigator node for TF, not the module node!
        # The navigator is in the robot's namespace (e.g., /robot_1) so its
        # TF listener subscribes to /robot_1/tf where the robot publishes.
        # The module node is in global namespace for COVEN protocol topics.
        self.tf_buffer = Buffer()
        # Use navigator (which is namespaced) for TF subscriptions
        tf_node = navigator if robot_namespace else node
        self.tf_listener = TransformListener(self.tf_buffer, tf_node)

        # Frame names are NOT prefixed - TurtleBot4 uses plain names
        # The namespace isolation happens via TF topic (/robot_1/tf)
        self.base_frame = "base_link"
        self.map_frame = "map"

        # Subscribe to map updates from SLAM
        # Multi-robot: subscribe to namespaced map topic (e.g., /robot_1/map)
        # Single-robot: subscribe to global /map
        if robot_namespace:
            map_topic = f'/{robot_namespace}/map'
        else:
            map_topic = '/map'

        self.map_sub = node.create_subscription(
            OccupancyGrid,
            map_topic,
            self._map_callback,
            10
        )

        tf_source = f"/{robot_namespace}/tf" if robot_namespace else "/tf"
        self.node.get_logger().info(
            f"Explorer initialized (base_frame: {self.base_frame}, "
            f"map_frame: {self.map_frame}, map_topic: {map_topic}, tf: {tf_source})"
        )

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

    def get_current_pose(self) -> Optional[PoseStamped]:
        """
        Get robot's current pose via TF2 lookup.

        Uses Time(seconds=0) to get the latest available transform, avoiding
        extrapolation errors when transforms haven't propagated yet.

        Returns:
            PoseStamped with current robot position, or None if lookup fails
        """
        try:
            # Use Time(seconds=0) for latest available transform
            # This avoids ExtrapolationException when current time transform
            # hasn't arrived yet (common in multi-robot scenarios)
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(seconds=0),  # Latest available
                timeout=Duration(seconds=1.0)
            )

            # Convert transform to PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose

        except TransformException as e:
            self.node.get_logger().warn(
                f"TF lookup failed ({self.map_frame} → {self.base_frame}): {e}"
            )
            return None

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

    def explore(self, duration: float = EXPLORATION_TIMEOUT, feedback_callback=None) -> Tuple[bool, dict]:
        """
        Execute exploration mission.

        Args:
            duration: Maximum exploration time in seconds
            feedback_callback: Optional callback function(coverage, frontiers) for periodic updates

        Returns:
            Tuple of (success, metrics_dict)
        """
        self.start_time = time.time()
        self.start_pose = self.get_current_pose()
        self.last_pose = self.start_pose  # Initialize for distance tracking

        if self.start_pose:
            self.node.get_logger().info(
                f"Starting exploration at ({self.start_pose.pose.position.x:.2f}, "
                f"{self.start_pose.pose.position.y:.2f}) (max duration: {duration}s)"
            )
        else:
            self.node.get_logger().warn(
                f"Starting exploration - could not get initial pose (max duration: {duration}s)"
            )

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

            # Call feedback callback if provided
            if feedback_callback:
                try:
                    feedback_callback(coverage, len(self.find_frontiers()))
                except Exception as e:
                    self.node.get_logger().warn(f"Feedback callback failed: {e}")

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

            # Small delay to let action client initialize
            time.sleep(0.5)

            # Wait for navigation to complete by polling with timeout
            timeout_duration = 30.0  # 30 seconds per goal
            start = time.time()
            while not self.navigator.isTaskComplete():
                if time.time() - start > timeout_duration:
                    self.node.get_logger().warn("Navigation goal timeout")
                    self.navigator.cancelTask()
                    break
                time.sleep(0.2)

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.node.get_logger().info("Reached frontier waypoint")
                total_distance += self._update_distance_traveled()
            elif result == TaskResult.CANCELED:
                self.node.get_logger().warn("Navigation canceled")
                break
            elif result == TaskResult.FAILED:
                self.node.get_logger().warn("Navigation failed, trying next frontier")

        # Compile metrics
        final_coverage = self._calculate_coverage()
        metrics = {
            "duration": float(time.time() - self.start_time),
            "iterations": int(iteration),
            "coverage": float(final_coverage),
            "explored_cells": int(self.explored_cells),  # Convert numpy int64 to Python int
            "distance_traveled": float(total_distance)
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
            time.sleep(0.1)

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
        """
        Select the closest goal from a list based on Euclidean distance.

        Args:
            goals: List of candidate goal poses

        Returns:
            Closest goal pose, or None if no goals available
        """
        if not goals:
            return None

        # Get current robot position
        current_pose = self.get_current_pose()
        if not current_pose:
            # Fall back to first goal if can't get current pose
            self.node.get_logger().debug("Cannot get current pose, selecting first goal")
            return goals[0]

        robot_x = current_pose.pose.position.x
        robot_y = current_pose.pose.position.y

        # Find closest goal by Euclidean distance
        min_distance = float('inf')
        closest_goal = None

        for goal in goals:
            dx = goal.pose.position.x - robot_x
            dy = goal.pose.position.y - robot_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < min_distance:
                min_distance = distance
                closest_goal = goal

        return closest_goal

    def _distance_between_poses(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate Euclidean distance between two poses."""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _update_distance_traveled(self) -> float:
        """
        Calculate distance traveled since last pose update.

        Updates last_pose and returns the distance traveled.
        """
        current_pose = self.get_current_pose()
        if not current_pose:
            return 0.0

        distance = 0.0
        if self.last_pose:
            distance = self._distance_between_poses(self.last_pose, current_pose)

        self.last_pose = current_pose
        return distance
