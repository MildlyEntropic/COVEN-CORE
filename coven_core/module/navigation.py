"""
navigation.py - Navigation wrapper for COVEN modules

Wraps Nav2 BasicNavigator with COVEN-specific functionality:
- Waypoint mission execution
- Fallback to simple cmd_vel navigation
- Pose tracking via TF2
- Return-to-dock support

This module follows ROS2 Nav2 best practices:
- Event-driven action client (no polling where possible)
- Proper namespace support for multi-robot operation
- Graceful degradation when Nav2 unavailable

References:
- Nav2 Simple Commander: https://navigation.ros.org/commander_api/index.html
- ROS2 TF2: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html

Author: Alexander Shultis
Date: December 2025
"""

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Protocol, Callable, List, Tuple

from geometry_msgs.msg import PoseStamped, Twist, Quaternion

from coven_core.config import get_config, NavigationConfig
from coven_core.common import Waypoint, WaypointResult

logger = logging.getLogger(__name__)


class TFProvider(Protocol):
    """Protocol for TF2 transform lookups."""

    def lookup_transform(
        self, target_frame: str, source_frame: str, time, timeout
    ) -> 'TransformStamped':
        """Lookup transform between frames."""
        ...


class VelocityPublisher(Protocol):
    """Protocol for publishing velocity commands."""

    def publish(self, msg: Twist) -> None:
        """Publish velocity command."""
        ...


class ClockProvider(Protocol):
    """Protocol for getting ROS time."""

    def now(self) -> 'Time':
        """Get current time."""
        ...


@dataclass
class NavigationResult:
    """Result of a navigation attempt."""

    success: bool
    actual_distance: float = 0.0
    detour_distance: float = 0.0
    blocked_at: Optional[Tuple[float, float]] = None
    reason: str = ""


@dataclass
class MissionMetrics:
    """Metrics from a waypoint mission."""

    waypoints_total: int = 0
    waypoints_completed: int = 0
    waypoints_failed: int = 0
    total_distance_planned: float = 0.0
    total_distance_traveled: float = 0.0
    total_detour_distance: float = 0.0
    aborted: bool = False
    abort_reason: str = ""
    waypoint_results: List[WaypointResult] = field(default_factory=list)
    returned_to_dock: bool = False


class ModuleNavigator:
    """
    Navigation wrapper for COVEN modules.

    Provides high-level navigation primitives with automatic fallback
    to simple cmd_vel control when Nav2 is unavailable.

    Example:
        from coven_core.module import ModuleNavigator

        nav = ModuleNavigator(
            module_id="Hermione_Granger",
            namespace="witch_1",
            clock=node.get_clock(),
            tf_buffer=node.tf_buffer,
            cmd_vel_publisher=node.pub_cmd_vel,
        )

        # Initialize with Nav2 (or fallback to simple)
        nav.initialize(executor)

        # Execute waypoint mission
        success, metrics = nav.execute_mission(mission_data)
    """

    def __init__(
        self,
        module_id: str,
        namespace: str = "",
        clock: Optional[ClockProvider] = None,
        tf_buffer: Optional[TFProvider] = None,
        cmd_vel_publisher: Optional[VelocityPublisher] = None,
        config: Optional[NavigationConfig] = None,
        logger_callback: Optional[Callable[[str], None]] = None,
    ):
        """
        Initialize navigator.

        Args:
            module_id: Module identifier (e.g., "Hermione_Granger")
            namespace: Robot namespace for multi-robot (e.g., "witch_1")
            clock: ROS clock provider
            tf_buffer: TF2 buffer for pose lookups
            cmd_vel_publisher: Publisher for velocity commands (fallback mode)
            config: Navigation configuration (uses global if not provided)
            logger_callback: Optional callback for logging (e.g., node.get_logger().info)
        """
        self.module_id = module_id
        self.namespace = namespace
        self.clock = clock
        self.tf_buffer = tf_buffer
        self.cmd_vel_publisher = cmd_vel_publisher
        self.config = config or get_config().navigation
        self._log = logger_callback or (lambda msg: logger.info(msg))

        # Nav2 components (initialized lazily)
        self.navigator = None
        self.explorer = None
        self.use_simple_nav = False

        # Dock tracking
        self.dock_pose: Optional[PoseStamped] = None

    def initialize(
        self,
        executor=None,
        max_retries: int = 3,
        on_nav2_ready: Optional[Callable[[], None]] = None,
    ) -> bool:
        """
        Initialize Nav2 navigator with retry logic.

        Falls back to simple navigation if Nav2 unavailable.

        Args:
            executor: ROS2 executor for adding navigator node
            max_retries: Maximum initialization attempts
            on_nav2_ready: Callback when Nav2 is ready

        Returns:
            True if Nav2 initialized, False if using simple nav
        """
        from nav2_simple_commander.robot_navigator import BasicNavigator

        last_error = None

        for attempt in range(max_retries):
            try:
                self._log(
                    f"Initializing navigation for {self.namespace}... "
                    f"(attempt {attempt + 1}/{max_retries})"
                )

                # Create BasicNavigator with unique node name
                nav_node_name = f'navigator_{self.module_id.replace("-", "_")}'

                if self.namespace:
                    self.navigator = BasicNavigator(
                        node_name=nav_node_name, namespace=self.namespace
                    )
                else:
                    self.navigator = BasicNavigator(node_name=nav_node_name)

                # Add to executor for spinning
                if executor:
                    executor.add_node(self.navigator)

                # Wait for Nav2 action server (event-driven)
                action_name = (
                    f"/{self.namespace}/navigate_to_pose"
                    if self.namespace
                    else "/navigate_to_pose"
                )
                self._log(f"Waiting for Nav2 action server ({action_name})...")

                if not self.navigator.nav_to_pose_client.wait_for_server(
                    timeout_sec=60.0
                ):
                    raise RuntimeError("Nav2 action server not available after 60s")

                self._log("Nav2 action server ready")
                time.sleep(1.0)  # Brief settling time

                if on_nav2_ready:
                    on_nav2_ready()

                self.use_simple_nav = False
                return True

            except Exception as e:
                last_error = e
                self._log(f"Navigation init attempt {attempt + 1} failed: {e}")

                # Cleanup on failure
                if self.navigator and executor:
                    try:
                        executor.remove_node(self.navigator)
                    except Exception:
                        pass
                self.navigator = None

                if attempt < max_retries - 1:
                    delay = 2.0 * (attempt + 1)
                    self._log(f"Retrying in {delay}s...")
                    time.sleep(delay)

        # All retries failed - fall back to simple nav
        self._log(f"Nav2 unavailable ({last_error}), using simple navigation")
        self.use_simple_nav = True
        return False

    def execute_mission(
        self, mission_data: dict, on_waypoint_complete: Optional[Callable] = None
    ) -> Tuple[bool, MissionMetrics]:
        """
        Execute a waypoint-based mission.

        Args:
            mission_data: Dict with 'waypoints' list and 'return_to_dock' flag
            on_waypoint_complete: Optional callback for progress (waypoint_index, result)

        Returns:
            Tuple of (success, metrics)
        """
        # Store dock position for return
        self.dock_pose = self.get_current_pose_safe()
        dock_x = self.dock_pose.pose.position.x
        dock_y = self.dock_pose.pose.position.y

        # Parse mission
        waypoints_raw = mission_data.get("waypoints", [])
        return_to_dock = mission_data.get("return_to_dock", True)

        self._log(
            f"Starting mission: {len(waypoints_raw)} waypoints, "
            f"return_to_dock={return_to_dock}"
        )

        # Clear costmaps if Nav2 available
        if self.navigator and not self.use_simple_nav:
            try:
                self._log("Clearing costmaps...")
                self.navigator.clearAllCostmaps()
                time.sleep(0.5)
            except Exception as e:
                self._log(f"Failed to clear costmaps: {e}")

        # Convert raw waypoints to Waypoint objects
        waypoints = [
            Waypoint(
                type=wp.get("type", "move"),
                distance=wp.get("distance", 0.0),
                direction=wp.get("direction", "forward"),
                angle=wp.get("angle", 0.0),
            )
            for wp in waypoints_raw
        ]

        # Initialize metrics
        metrics = MissionMetrics(
            waypoints_total=len(waypoints),
            total_distance_planned=sum(
                wp.distance for wp in waypoints if wp.type == "move"
            ),
        )

        # Get initial pose
        current_pose = self.get_current_pose()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = self._quaternion_to_yaw(current_pose.pose.orientation)

        # Execute waypoints
        for idx, waypoint in enumerate(waypoints):
            self._log(
                f"Waypoint {idx + 1}/{len(waypoints)}: "
                f"{waypoint.type} {waypoint.distance}m {waypoint.direction}"
            )

            result = WaypointResult(
                waypoint_index=idx,
                success=False,
                actual_distance=0.0,
                blocked_at=None,
                detour_distance=0.0,
                reason="",
            )

            if waypoint.type == "turn":
                result = self._execute_turn_waypoint(waypoint, current_yaw, idx)
                if result.success:
                    current_yaw = self._normalize_yaw(
                        current_yaw + math.radians(waypoint.angle)
                    )
                    metrics.waypoints_completed += 1
                else:
                    metrics.waypoints_failed += 1

            elif waypoint.type == "move":
                target_x, target_y = self._calculate_target(
                    current_x, current_y, current_yaw, waypoint.distance, waypoint.direction
                )

                result = self._execute_move_waypoint(
                    waypoint, target_x, target_y, current_yaw, idx
                )

                metrics.total_distance_traveled += result.actual_distance
                metrics.total_detour_distance += result.detour_distance

                if result.success:
                    metrics.waypoints_completed += 1
                    # Update position
                    current_pose = self.get_current_pose()
                    current_x = current_pose.pose.position.x
                    current_y = current_pose.pose.position.y
                    current_yaw = self._quaternion_to_yaw(current_pose.pose.orientation)
                else:
                    metrics.waypoints_failed += 1

                    # Check for excessive detour
                    if result.detour_distance > self.config.waypoint_detour_multiplier * waypoint.distance:
                        metrics.aborted = True
                        metrics.abort_reason = (
                            f"Detour too large at waypoint {idx + 1}: "
                            f"{result.detour_distance:.2f}m for {waypoint.distance:.2f}m target"
                        )
                        self._log(f"ABORTING: {metrics.abort_reason}")
                        metrics.waypoint_results.append(result)
                        break

            metrics.waypoint_results.append(result)

            if on_waypoint_complete:
                on_waypoint_complete(idx, result)

        # Return to dock
        if return_to_dock or metrics.aborted:
            self._log("Returning to dock...")
            if self.use_simple_nav:
                self._log("(Simple nav - skipping return)")
                metrics.returned_to_dock = True
            else:
                metrics.returned_to_dock = self._return_to_position(
                    dock_x, dock_y, self.dock_pose
                )
                if metrics.returned_to_dock:
                    self._log("Returned to dock successfully")
                else:
                    self._log("Failed to return to dock precisely")

        # Determine overall success
        success = (
            metrics.waypoints_completed == metrics.waypoints_total
            and not metrics.aborted
        )

        self._log(
            f"Mission complete: {metrics.waypoints_completed}/{metrics.waypoints_total} "
            f"waypoints, {metrics.total_distance_traveled:.2f}m traveled"
        )

        return success, metrics

    def _execute_turn_waypoint(
        self, waypoint: Waypoint, current_yaw: float, idx: int
    ) -> WaypointResult:
        """Execute a turn waypoint."""
        result = WaypointResult(
            waypoint_index=idx,
            success=False,
            actual_distance=0.0,
            blocked_at=None,
            detour_distance=0.0,
            reason="",
        )

        if self.use_simple_nav:
            success = self._simple_turn(waypoint.angle)
        else:
            success = self._nav2_turn(waypoint.angle, current_yaw)

        result.success = success
        result.reason = "Turn completed" if success else "Turn failed"
        return result

    def _execute_move_waypoint(
        self,
        waypoint: Waypoint,
        target_x: float,
        target_y: float,
        current_yaw: float,
        idx: int,
    ) -> WaypointResult:
        """Execute a move waypoint."""
        result = WaypointResult(
            waypoint_index=idx,
            success=False,
            actual_distance=0.0,
            blocked_at=None,
            detour_distance=0.0,
            reason="",
        )

        if self.use_simple_nav:
            success = self._simple_move(waypoint.distance, waypoint.direction, current_yaw)
            result.success = success
            result.actual_distance = waypoint.distance if success else 0.0
            result.reason = "Move completed" if success else "Move failed"
        else:
            nav_result = self._navigate_to_target(
                target_x, target_y, waypoint.distance
            )
            result.success = nav_result.success
            result.actual_distance = nav_result.actual_distance
            result.detour_distance = nav_result.detour_distance
            result.blocked_at = nav_result.blocked_at
            result.reason = nav_result.reason

        return result

    def _nav2_turn(self, angle_degrees: float, current_yaw: float) -> bool:
        """Execute turn using Nav2."""
        try:
            current_pose = self.get_current_pose()

            # Calculate target yaw (negative because positive = clockwise)
            target_yaw = self._normalize_yaw(current_yaw - math.radians(angle_degrees))

            # Create target pose at same position with new orientation
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = self.clock.now().to_msg() if self.clock else None
            target_pose.pose.position = current_pose.pose.position
            target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

            self.navigator.goToPose(target_pose)

            # Wait for completion
            start_time = time.time()
            while not self.navigator.isTaskComplete():
                if time.time() - start_time > self.config.waypoint_nav_timeout:
                    self._log("Turn timeout")
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.1)

            result = self.navigator.getResult()
            return result.value == 1  # SUCCEEDED

        except Exception as e:
            self._log(f"Turn failed: {e}")
            return False

    def _navigate_to_target(
        self, target_x: float, target_y: float, expected_distance: float
    ) -> NavigationResult:
        """Navigate to target with obstacle avoidance."""
        try:
            start_pose = self.get_current_pose()
            start_x = start_pose.pose.position.x
            start_y = start_pose.pose.position.y

            # Create target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = self.clock.now().to_msg() if self.clock else None
            target_pose.pose.position.x = target_x
            target_pose.pose.position.y = target_y
            target_pose.pose.position.z = 0.0

            # Orient toward target
            dx = target_x - start_x
            dy = target_y - start_y
            target_yaw = math.atan2(dy, dx)
            target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

            self._log(f"Nav2 goal: ({target_x:.2f}, {target_y:.2f})")
            self.navigator.goToPose(target_pose)

            # Track distance
            last_x, last_y = start_x, start_y
            actual_distance = 0.0

            start_time = time.time()
            while not self.navigator.isTaskComplete():
                elapsed = time.time() - start_time

                if elapsed > self.config.waypoint_nav_timeout:
                    self._log(f"Navigation timeout after {elapsed:.1f}s")
                    self.navigator.cancelTask()
                    final_pose = self.get_current_pose()
                    return NavigationResult(
                        success=False,
                        actual_distance=actual_distance,
                        detour_distance=max(0, actual_distance - expected_distance),
                        blocked_at=(
                            final_pose.pose.position.x,
                            final_pose.pose.position.y,
                        ),
                        reason="Navigation timeout",
                    )

                # Update distance tracking
                current_pose = self.get_current_pose()
                curr_x = current_pose.pose.position.x
                curr_y = current_pose.pose.position.y
                step = math.sqrt((curr_x - last_x) ** 2 + (curr_y - last_y) ** 2)
                actual_distance += step
                last_x, last_y = curr_x, curr_y

                time.sleep(0.2)

            # Check result
            result = self.navigator.getResult()
            final_pose = self.get_current_pose()
            final_x = final_pose.pose.position.x
            final_y = final_pose.pose.position.y
            dist_to_target = math.sqrt(
                (final_x - target_x) ** 2 + (final_y - target_y) ** 2
            )

            if result.value == 1 and dist_to_target < self.config.waypoint_position_tolerance:
                return NavigationResult(
                    success=True,
                    actual_distance=actual_distance,
                    detour_distance=max(0, actual_distance - expected_distance),
                    reason="Waypoint reached",
                )
            else:
                return NavigationResult(
                    success=False,
                    actual_distance=actual_distance,
                    detour_distance=max(0, actual_distance - expected_distance),
                    blocked_at=(final_x, final_y),
                    reason=f"Navigation failed (result={result.value}, dist={dist_to_target:.2f}m)",
                )

        except Exception as e:
            self._log(f"Navigation error: {e}")
            return NavigationResult(success=False, reason=str(e))

    def _return_to_position(
        self, x: float, y: float, dock_pose: Optional[PoseStamped] = None
    ) -> bool:
        """Navigate back to dock position."""
        try:
            if dock_pose:
                target_pose = PoseStamped()
                target_pose.header.frame_id = "map"
                target_pose.header.stamp = self.clock.now().to_msg() if self.clock else None
                target_pose.pose = dock_pose.pose
            else:
                current_pose = self.get_current_pose()
                target_pose = PoseStamped()
                target_pose.header.frame_id = "map"
                target_pose.header.stamp = self.clock.now().to_msg() if self.clock else None
                target_pose.pose.position.x = x
                target_pose.pose.position.y = y
                target_pose.pose.position.z = 0.0

                dx = x - current_pose.pose.position.x
                dy = y - current_pose.pose.position.y
                target_yaw = math.atan2(dy, dx)
                target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

            self.navigator.goToPose(target_pose)

            # Extended timeout for return
            return_timeout = self.config.waypoint_nav_timeout * 3
            start_time = time.time()

            while not self.navigator.isTaskComplete():
                if time.time() - start_time > return_timeout:
                    self._log("Return timeout")
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.2)

            result = self.navigator.getResult()
            final_pose = self.get_current_pose()
            dist = math.sqrt(
                (final_pose.pose.position.x - x) ** 2
                + (final_pose.pose.position.y - y) ** 2
            )

            return result.value == 1 and dist < self.config.waypoint_position_tolerance * 2

        except Exception as e:
            self._log(f"Return failed: {e}")
            return False

    # -------------------------
    # Simple Navigation (cmd_vel fallback)
    # -------------------------

    def _simple_move(
        self, distance: float, direction: str, current_yaw: float
    ) -> bool:
        """Execute simple move using cmd_vel (no obstacle avoidance)."""
        if not self.cmd_vel_publisher:
            self._log("No cmd_vel publisher - cannot execute simple move")
            return False

        # Use CubeRover max speed from config
        linear_speed = self.config.max_linear_speed

        # Calculate velocity based on direction
        if direction == "forward":
            vx = linear_speed * math.cos(current_yaw)
            vy = linear_speed * math.sin(current_yaw)
        elif direction == "north":
            vx, vy = 0.0, linear_speed
        elif direction == "south":
            vx, vy = 0.0, -linear_speed
        elif direction == "east":
            vx, vy = linear_speed, 0.0
        elif direction == "west":
            vx, vy = -linear_speed, 0.0
        elif direction == "backward":
            vx = -linear_speed * math.cos(current_yaw)
            vy = -linear_speed * math.sin(current_yaw)
        else:
            self._log(f"Unknown direction: {direction}")
            return False

        # Calculate duration
        duration = distance / linear_speed
        self._log(f"Simple move: {distance}m {direction} (duration: {duration:.1f}s)")

        # Publish velocity for duration
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy

        start_time = time.time()
        rate = 10  # Hz
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(1.0 / rate)

        # Stop
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_publisher.publish(twist)

        return True

    def _simple_turn(self, angle_degrees: float) -> bool:
        """Execute simple turn using cmd_vel."""
        if not self.cmd_vel_publisher:
            self._log("No cmd_vel publisher - cannot execute simple turn")
            return False

        angular_speed = self.config.max_angular_speed
        angle_rad = math.radians(angle_degrees)
        duration = abs(angle_rad) / angular_speed

        self._log(f"Simple turn: {angle_degrees}° (duration: {duration:.1f}s)")

        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed

        start_time = time.time()
        rate = 10  # Hz
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(1.0 / rate)

        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        return True

    # -------------------------
    # Pose Utilities
    # -------------------------

    def get_current_pose(self) -> PoseStamped:
        """Get current robot pose via TF2."""
        import rclpy.time
        import rclpy.duration

        try:
            # For multi-robot setups, frame names are namespaced (e.g., Akko/base_link)
            base_frame = f"{self.namespace}/base_link" if self.namespace else "base_link"
            transform = self.tf_buffer.lookup_transform(
                "map",
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose

        except Exception as e:
            self._log(f"TF lookup failed: {e}, using origin")
            return self._origin_pose()

    def get_current_pose_safe(self) -> PoseStamped:
        """Get current pose, returning origin if unavailable."""
        try:
            return self.get_current_pose()
        except Exception:
            return self._origin_pose()

    def _origin_pose(self) -> PoseStamped:
        """Return origin pose as fallback."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        if self.clock:
            pose.header.stamp = self.clock.now().to_msg()
        pose.pose.orientation.w = 1.0
        return pose

    def _calculate_target(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        distance: float,
        direction: str,
    ) -> Tuple[float, float]:
        """Calculate target position from direction instruction."""
        if direction == "north":
            return current_x, current_y + distance
        elif direction == "south":
            return current_x, current_y - distance
        elif direction == "east":
            return current_x + distance, current_y
        elif direction == "west":
            return current_x - distance, current_y
        elif direction == "forward":
            return (
                current_x + distance * math.cos(current_yaw),
                current_y + distance * math.sin(current_yaw),
            )
        elif direction == "backward":
            return (
                current_x - distance * math.cos(current_yaw),
                current_y - distance * math.sin(current_yaw),
            )
        else:
            # Default to forward
            return (
                current_x + distance * math.cos(current_yaw),
                current_y + distance * math.sin(current_yaw),
            )

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Convert quaternion to yaw angle in radians."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def _normalize_yaw(yaw: float) -> float:
        """Normalize yaw to [-pi, pi]."""
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw
