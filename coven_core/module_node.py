"""
module_node.py — COVEN Phase 1

ROS2 node representing a single COVEN-compliant module (e.g., ReconRover).
Implements the module side of the plug-level FSM lifecycle:

    BOOT → IDENTIFY → WAIT_VERIFY → NORMAL → FIELD_OPS → NORMAL

Responsibilities:
- Respond to IDENTIFY_REQ with module ID/type/firmware.
- Respond to VERIFY_REQ with VerifyRep (OK or FAIL).
- React to power enable messages (+12V rail).
- Publish periodic heartbeat while in NORMAL.
- Respond to TASK_REQ with TASK_ACK, emit TASK_START,
  simulate task, and emit TASK_COMPLETE.

Author: Alexander Shultis
Date: September 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
# --- Standard library ---
import base64
import gzip
import json
import os
import subprocess
import threading
import time
import uuid
from typing import Optional

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# --- Navigation ---
from nav2_simple_commander.robot_navigator import BasicNavigator

# --- COVEN action interfaces ---
from coven_interfaces.action import ExecuteTask

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import (
    ModuleState, BidNotice, BidProposal, Waypoint, WaypointResult,
    CoverageGoal, CoverageStatus, BatteryConfig, MissionRequest
)
from coven_core.serialization import encode, decode
from coven_core.exploration import Explorer
from coven_core.config import get_config
from coven_core.module.bidding import BidCalculator
import math


# ------------------------
# --- Constants ---
# ------------------------
# NOTE: These are now ROS2 parameters - defaults defined here for reference
# Actual values loaded from config/coven_params.yaml
DEFAULT_HB_PERIOD = 0.8  # seconds between heartbeats
DEFAULT_TASK_TIMEOUT = 300.0  # seconds - watchdog timer for task execution
DEFAULT_MAP_STORAGE_DIR = '~/coven_maps'  # temporary map storage

# Waypoint navigation constants
WAYPOINT_DETOUR_MULTIPLIER = 10.0  # Abort if detour > N * original distance
WAYPOINT_NAV_TIMEOUT = 60.0  # seconds per waypoint navigation attempt
WAYPOINT_POSITION_TOLERANCE = 0.3  # meters - how close to target counts as "arrived"


# ------------------------
# --- Module Node ---
# ------------------------
class Module(Node):
    """Module node that manages the FSM lifecycle for a single module."""

    def __init__(self, module_id=None, module_type="ReconRover", fw="0.0.1", executor=None, robot_namespace=""):
        super().__init__('coven_module')

        # Declare ROS2 parameters (loaded from config/coven_params.yaml)
        self.declare_parameter('heartbeat_period', DEFAULT_HB_PERIOD)
        self.declare_parameter('task_timeout', DEFAULT_TASK_TIMEOUT)
        self.declare_parameter('map_storage_dir', DEFAULT_MAP_STORAGE_DIR)
        self.declare_parameter('skip_health_check', False)  # For testing without hardware

        # Exploration parameters
        self.declare_parameter('exploration.frontier_radius', 3.0)
        self.declare_parameter('exploration.min_frontier_size', 10)
        self.declare_parameter('exploration.coverage_threshold', 0.80)
        self.declare_parameter('exploration.timeout', 300.0)
        self.declare_parameter('exploration.no_frontier_limit', 3)
        self.declare_parameter('exploration.nav_timeout', 60.0)

        # Get parameter values
        self.hb_period = self.get_parameter('heartbeat_period').value
        self.task_timeout = self.get_parameter('task_timeout').value
        self.map_storage_dir = os.path.expanduser(self.get_parameter('map_storage_dir').value)
        self.skip_health_check = self.get_parameter('skip_health_check').value

        # Use provided module_id, or generate a witch name for this module
        # The witch naming system gives each rover a unique, memorable name
        self.module_id = module_id or common.get_witch_name()
        self.module_type = module_type
        self.fw = fw
        self.state = ModuleState.BOOT
        self.seq = 0
        self.hb_timer = None
        self.task_watchdog_timer = None  # Watchdog to prevent stuck tasks
        self.executor = executor  # Store executor reference to add navigator node
        self.robot_namespace = robot_namespace  # Namespace for robot-specific topics

        # TF2 for pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Navigation components (initialized on demand)
        self.navigator = None
        self.explorer = None
        self.dock_pose = None  # Store initial pose for return

        # Health monitoring
        self.health_status = {
            'lidar': True,
            'camera': True,
            'nav2': False,  # Not initialized until needed
            'slam': False   # Not initialized until needed
        }
        self.last_sensor_check = self.get_clock().now()

        # Hardware interface (persistent instance for battery tracking)
        try:
            from coven_core.simulated_hardware import SimulatedHardware
            self.hw = SimulatedHardware(module_id=self.module_id, randomize_battery=True)
            self.get_logger().info(f"Battery: {self.hw.get_battery_percentage()*100:.0f}%")
        except Exception as e:
            self.hw = None
            self.get_logger().warn(f"Hardware init failed: {e}")

        # Bid calculator (uses extracted module for cost calculation)
        self._bid_calculator = BidCalculator(
            module_type=self.module_type,
            hardware=self.hw,
            pose_provider=self,  # Module implements PoseProvider protocol
            config=get_config().bidding,
        )

        # ROS Topics - Use absolute paths (/coven/...) so they work globally
        # regardless of node namespace. This allows namespaced modules to communicate
        # with the non-namespaced dock node.
        self.sub_ident_req = self.create_subscription(String, '/coven/identify_req', self.on_ident_req, 10)
        self.pub_ident_rep = self.create_publisher(String, '/coven/identify_rep', 10)

        self.sub_verify_req = self.create_subscription(String, '/coven/verify_req', self.on_verify_req, 10)
        self.pub_verify_rep = self.create_publisher(String, '/coven/verify_rep', 10)

        self.sub_enable_12v = self.create_subscription(String, '/coven/enable_12v', self.on_power, 10)

        self.pub_hb = self.create_publisher(String, '/coven/heartbeat', 10)

        self.sub_task_req = self.create_subscription(String, '/coven/task_req', self.on_task_req, 10)
        self.pub_task_ack = self.create_publisher(String, '/coven/task_ack', 10)
        self.pub_task_start = self.create_publisher(String, '/coven/task_start', 10)
        self.pub_task_complete = self.create_publisher(String, '/coven/task_complete', 10)

        # Bidding system
        self.sub_bid_notice = self.create_subscription(String, '/coven/bid_notice', self.on_bid_notice, 10)
        self.pub_bid_proposal = self.create_publisher(String, '/coven/bid_proposal', 10)
        self.last_task_complete_time = time.time()  # Track idle time for cost calculation

        # Module ready announcement (triggers immediate IDENTIFY from dock)
        self.pub_module_ready = self.create_publisher(String, '/coven/module_ready', 10)

        # Coverage exploration status (periodic updates during coverage missions)
        self.pub_coverage_status = self.create_publisher(String, '/coven/coverage_status', 10)
        self._coverage_status_timer = None  # Timer for periodic status updates
        self._coverage_mission_start_time = 0.0  # Track mission duration
        self._coverage_distance_traveled = 0.0  # Track distance for battery drain

        # Direct velocity control (fallback when Nav2 not available)
        # Uses /cmd_vel which bridges to Gazebo's /model/witch_1/cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.use_simple_nav = False  # Set to True if Nav2 unavailable

        # Action server for task execution (modern alternative to topic-based approach)
        # Replace hyphens with underscores for valid topic name
        action_name = f'coven/{self.module_id.replace("-", "_")}/execute_task'
        self._action_server = ActionServer(
            self,
            ExecuteTask,
            action_name,
            execute_callback=self._execute_action_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        self._current_goal_handle = None
        self._cancel_requested = False

        self.get_logger().info(f"Module {self.module_id} booted on 5V — waiting for IDENTIFY.")

        # Announce module ready after delay (give bridges time to fully connect)
        # Bridges take ~2-3 seconds to establish, so we wait 3 seconds before announcing
        self._ready_announce_timer = None
        self._ready_retry_count = 0
        self._max_ready_retries = 5  # Retry up to 5 times (15 seconds total)
        self.create_timer(3.0, self._announce_ready, callback_group=None)

    # ------------------------
    # MODULE READY ANNOUNCEMENT
    # ------------------------
    def _announce_ready(self):
        """
        Announce module is ready and waiting for IDENTIFY.

        This method retries announcing until IDENTIFY is received or max retries reached.
        Bridges take time to fully connect, so we may need multiple attempts.
        """
        if self.state != ModuleState.BOOT:
            # Already identified, cancel any pending retry timer
            if self._ready_announce_timer:
                self._ready_announce_timer.cancel()
                self._ready_announce_timer = None
            return

        self._ready_retry_count += 1

        # Publish ready announcement to trigger dock's immediate IDENTIFY
        ready_msg = json.dumps({"module_id": self.module_id, "state": "BOOT"})
        self.pub_module_ready.publish(String(data=ready_msg))

        if self._ready_retry_count == 1:
            self.get_logger().info(f"Announced ready — waiting for IDENTIFY from dock")
        else:
            self.get_logger().info(
                f"Re-announced ready (attempt {self._ready_retry_count}/{self._max_ready_retries}) — "
                f"waiting for IDENTIFY from dock"
            )

        # Schedule retry if we haven't exceeded max retries
        if self._ready_retry_count < self._max_ready_retries:
            # Retry every 3 seconds until identified or max retries
            self._ready_announce_timer = self.create_timer(
                3.0, self._announce_ready, callback_group=None
            )
        else:
            self.get_logger().warn(
                f"Max ready announcements reached ({self._max_ready_retries}). "
                f"If no dock responds, module will wait for periodic IDENTIFY broadcasts."
            )

    # ------------------------
    # IDENTIFY / VERIFY
    # ------------------------
    def on_ident_req(self, msg: String):
        if self.state != ModuleState.BOOT:
            return
        req = common.ident_req_decode(msg)
        if not req:
            return

        # Cancel ready announcement retry timer since we've been identified
        if self._ready_announce_timer:
            self._ready_announce_timer.cancel()
            self._ready_announce_timer = None

        self.get_logger().info("IDENTIFY_REQ received → responding with module ID")
        rep = common.IdentifyRep(
            req_id=req.req_id,
            module_id=self.module_id,
            module_type=self.module_type,
            fw=self.fw
        )
        self.pub_ident_rep.publish(String(data=common.ident_rep_encode(rep)))
        self.state = ModuleState.WAIT_VERIFY

    def on_verify_req(self, msg: String):
        if self.state != ModuleState.WAIT_VERIFY:
            return
        req = common.verify_req_decode(msg)
        if not req or req.module_id != self.module_id:
            return

        # Perform health check before verification
        health_ok, health_reason = self.check_health()

        if health_ok:
            self.get_logger().info("VERIFY_REQ matched → replying OK (health check passed)")
            rep = common.VerifyRep(module_id=self.module_id, ok=True, reason="")
        else:
            self.get_logger().warn(f"VERIFY_REQ matched → replying FAIL: {health_reason}")
            rep = common.VerifyRep(module_id=self.module_id, ok=False, reason=health_reason)

        self.pub_verify_rep.publish(String(data=common.verify_rep_encode(rep)))

    # ------------------------
    # HEALTH MONITORING
    # ------------------------
    def check_health(self) -> tuple[bool, str]:
        """
        Check the health of critical subsystems.

        Returns:
            Tuple of (health_ok: bool, reason: str)
            - health_ok: True if all critical systems functional
            - reason: Empty string if OK, otherwise describes failure
        """
        # Skip health check if parameter is set (for testing)
        if self.skip_health_check:
            self.get_logger().info("Health check skipped (skip_health_check=true)")
            return True, ""

        # Check for lidar data (scan topic)
        # Normalize namespace: ensure leading slash, no trailing slash
        ns = self.robot_namespace.strip('/')
        scan_topic = f'/{ns}/scan' if ns else '/scan'
        try:
            # Check topic existence with retry (DDS discovery can be slow)
            lidar_present = False
            for attempt in range(3):
                topic_list = self.get_topic_names_and_types()
                lidar_present = any(topic[0] == scan_topic for topic in topic_list)
                if lidar_present:
                    break
                if attempt < 2:
                    import time
                    time.sleep(0.5)  # Brief wait for DDS discovery
            self.health_status['lidar'] = lidar_present

            if not lidar_present:
                return False, f"Lidar offline - {scan_topic} topic not found"

        except Exception as e:
            self.get_logger().warning(f"Health check failed: {e}")
            return False, f"Health check error: {str(e)}"

        # All critical systems OK
        return True, ""

    # ------------------------
    # POWER ENABLE
    # ------------------------
    def on_power(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get("module_id") != self.module_id:
            return
        if d.get("data") and self.state == ModuleState.WAIT_VERIFY:
            self.state = ModuleState.NORMAL
            self.get_logger().info("+12V enabled → entering NORMAL state")
            self.start_heartbeat()

    # ------------------------
    # HEARTBEAT
    # ------------------------
    def start_heartbeat(self):
        if self.hb_timer:
            return
        self.hb_timer = self.create_timer(self.hb_period, self.send_heartbeat)
        self.get_logger().info(f"Heartbeat started for {self.module_id} (period: {self.hb_period}s)")

    def stop_heartbeat(self):
        if self.hb_timer:
            self.hb_timer.cancel()
            self.hb_timer = None
        self.get_logger().info(f"Heartbeat stopped for {self.module_id}")

    def send_heartbeat(self):
        self.seq += 1
        hb = common.Heartbeat(module_id=self.module_id, seq=self.seq)
        self.pub_hb.publish(String(data=common.hb_encode(hb)))

    # ------------------------
    # BIDDING SYSTEM
    # ------------------------
    def on_bid_notice(self, msg: String):
        """Handle incoming bid notice from dock - calculate cost and respond."""
        notice = common.bid_notice_decode(msg)
        if not notice:
            return

        # Only bid if we're in NORMAL state and available
        can_execute = True
        reason = ""

        if self.state != ModuleState.NORMAL:
            can_execute = False
            reason = f"Module not in NORMAL state (current: {self.state.name})"

        # Calculate bid cost (lower is better)
        cost = self._calculate_bid_cost(notice.task)

        # Create and send proposal
        proposal = BidProposal(
            task_id=notice.task_id,
            module_id=self.module_id,
            cost=cost,
            can_execute=can_execute,
            reason=reason
        )

        self.pub_bid_proposal.publish(String(data=common.bid_proposal_encode(proposal)))

        if can_execute:
            self.get_logger().info(
                f"Submitted bid for '{notice.task}' (task_id: {notice.task_id}, cost: {cost:.2f})"
            )
        else:
            self.get_logger().info(
                f"Declined bid for '{notice.task}': {reason}"
            )

    def _calculate_bid_cost(self, task: str) -> float:
        """
        Calculate bid cost for a task using the extracted BidCalculator.

        Delegates to coven_core.module.bidding.BidCalculator which handles:
        - Idle time bonus
        - Battery penalty
        - Dock obstruction penalty
        - Task type compatibility

        Returns:
            float: Cost value (lower is better, typically 0-100 range)
        """
        return self._bid_calculator.calculate_cost(task)

    # ------------------------
    # TASK WATCHDOG
    # ------------------------
    def _start_task_watchdog(self, task_name: str):
        """Start watchdog timer to prevent stuck tasks."""
        if self.task_watchdog_timer:
            self.task_watchdog_timer.cancel()

        timeout = self.task_timeout
        self.task_watchdog_timer = self.create_timer(
            timeout,
            lambda: self._on_task_timeout(task_name)
        )
        self.get_logger().info(f"Task watchdog started: {task_name} (timeout: {timeout}s)")

    def _stop_task_watchdog(self):
        """Stop watchdog timer after task completes."""
        if self.task_watchdog_timer:
            self.task_watchdog_timer.cancel()
            self.task_watchdog_timer = None
            self.get_logger().debug("Task watchdog stopped")

    def _on_task_timeout(self, task_name: str):
        """Handle task timeout - force return to NORMAL state."""
        self.get_logger().error(
            f"TASK TIMEOUT: {task_name} exceeded {self.task_timeout}s limit. "
            f"Forcing return to NORMAL state."
        )

        # Force state transition
        self.state = ModuleState.NORMAL
        self.start_heartbeat()

        # Emit task complete with failure
        tc = common.TaskComplete(
            module_id=self.module_id,
            task=task_name,
            success=False,
            note=f"Task timeout after {self.task_timeout}s",
            map_data="",
            map_yaml="",
            exploration_metrics={}
        )
        self.pub_task_complete.publish(String(data=common.task_complete_encode(tc)))

        # Stop the watchdog
        self._stop_task_watchdog()

    # ------------------------
    # TASK HANDLING
    # ------------------------
    def on_task_req(self, msg: String):
        req = common.task_req_decode(msg)
        if not req or req.module_id != self.module_id:
            return
        self.get_logger().info(f"TASK_REQ received: {req.task}")
        ack = common.TaskAck(module_id=self.module_id, accepted=True)
        self.pub_task_ack.publish(String(data=common.task_ack_encode(ack)))
        self.get_logger().info("Accepted task → preparing to undock")

        threading.Thread(target=self.execute_task, args=(req.task,), daemon=True).start()

    def execute_task(self, task_name):
        """Execute the assigned task (exploration mission)."""
        # Stop heartbeat and emit task start
        self.stop_heartbeat()
        self.state = ModuleState.FIELD_OPS

        # Start watchdog timer to prevent stuck tasks
        self._start_task_watchdog(task_name)

        ts = common.TaskStart(module_id=self.module_id, task=task_name)
        self.pub_task_start.publish(String(data=common.task_start_encode(ts)))
        self.get_logger().info(f"Executing FIELD_OPS for task: {task_name}")

        success = False
        map_data_b64 = ""
        map_yaml_b64 = ""
        metrics = {}

        try:
            # Check if this is a JSON mission (waypoint or coverage)
            is_waypoint_mission = False
            is_coverage_mission = False
            mission_data = None
            coverage_goal = None

            if task_name.startswith("{"):
                try:
                    mission_data = json.loads(task_name)
                    if mission_data.get("task") == "explore" and "waypoints" in mission_data:
                        is_waypoint_mission = True
                    elif mission_data.get("task") == "coverage" and "coverage_goal" in mission_data:
                        is_coverage_mission = True
                        # Decode coverage goal from mission data
                        cg = mission_data["coverage_goal"]
                        coverage_goal = CoverageGoal(
                            target_coverage=cg.get("target_coverage", 0.95),
                            sector=cg.get("sector"),
                            sector_bounds=tuple(cg["sector_bounds"]) if cg.get("sector_bounds") else None,
                            max_exploration_time=cg.get("max_exploration_time", 300.0),
                            return_on_low_battery=cg.get("return_on_low_battery", True),
                            battery_return_threshold=cg.get("battery_return_threshold", 0.20),
                        )
                except json.JSONDecodeError:
                    pass

            if is_waypoint_mission:
                # Execute waypoint-based exploration mission
                success, metrics = self._execute_waypoint_mission(mission_data)
                map_data_b64, map_yaml_b64 = "", ""  # Maps captured during navigation

            elif is_coverage_mission:
                # Execute coverage-based autonomous exploration
                # Initialize navigation first
                if not self.navigator:
                    try:
                        self._initialize_navigation()
                    except RuntimeError as e:
                        self.get_logger().error(f"Navigation initialization failed: {e}")
                        self._stop_task_watchdog()
                        self.state = ModuleState.NORMAL
                        self.start_heartbeat()
                        tc = common.TaskComplete(
                            module_id=self.module_id,
                            task=task_name,
                            success=False,
                            note="Nav2 not available for coverage mission",
                            map_data="",
                            map_yaml="",
                            exploration_metrics={}
                        )
                        self.pub_task_complete.publish(String(data=common.task_complete_encode(tc)))
                        return

                # Store dock position
                self.dock_pose = self._get_current_pose()
                self._publish_dock_transform(self.dock_pose)

                # Execute coverage mission
                success, metrics = self._execute_coverage_mission(coverage_goal)

                # Save and serialize map
                map_data_b64, map_yaml_b64 = self._save_and_serialize_map()

            elif "explore" in task_name.lower():
                # Initialize navigation if not already done
                if not self.navigator:
                    try:
                        self._initialize_navigation()
                    except RuntimeError as e:
                        self.get_logger().error(f"Navigation initialization failed: {e}")
                        self.get_logger().warn("Cannot execute exploration task without Nav2. Returning to dock.")
                        success = False

                        # Stop watchdog before early return
                        self._stop_task_watchdog()

                        # Early return to dock without exploration
                        self.state = ModuleState.NORMAL
                        self.start_heartbeat()

                        tc = common.TaskComplete(
                            module_id=self.module_id,
                            task=task_name,
                            success=False,
                            note="Nav2 not available - run in full mode (-#f) for exploration",
                            map_data="",
                            map_yaml="",
                            exploration_metrics={}
                        )
                        self.pub_task_complete.publish(String(data=common.task_complete_encode(tc)))
                        self.get_logger().info(f"Task {task_name} failed — returning to NORMAL state")
                        return

                # Store dock position for return and publish as TF
                self.dock_pose = self._get_current_pose()
                self._publish_dock_transform(self.dock_pose)

                # Execute exploration (use exploration.timeout parameter)
                explore_timeout = self.get_parameter('exploration.timeout').value
                success, metrics = self.explorer.explore(duration=explore_timeout)

                # Return to dock
                if success and self.dock_pose:
                    return_success = self.explorer.return_to_dock(self.dock_pose)
                    if not return_success:
                        self.get_logger().warn("Failed to return to dock, using best effort")

                # Save and serialize map
                map_data_b64, map_yaml_b64 = self._save_and_serialize_map()

            else:
                # Fallback to simple delay for other task types
                delay_time = self.get_parameter('exploration.timeout').value
                self.get_logger().info(f"Unknown task type '{task_name}', using delay simulation ({delay_time}s)")
                time.sleep(delay_time)
                success = True

        except Exception as e:
            self.get_logger().error(f"Task execution failed: {e}")
            success = False

        finally:
            # Stop watchdog timer (task completed or failed)
            self._stop_task_watchdog()

            # Return and emit task complete
            self.state = ModuleState.NORMAL
            self.start_heartbeat()
            self.last_task_complete_time = time.time()  # Update for bid cost calculation
            self._bid_calculator.mark_task_complete()  # Sync with bid calculator

            tc = common.TaskComplete(
                module_id=self.module_id,
                task=task_name,
                success=success,
                note=f"Exploration complete: {metrics.get('coverage', 0):.1%} coverage" if success else "Task failed",
                map_data=map_data_b64,
                map_yaml=map_yaml_b64,
                exploration_metrics=metrics
            )
            self.pub_task_complete.publish(String(data=common.task_complete_encode(tc)))
            self.get_logger().info(f"Task {task_name} complete — rejoined dock")

    # ------------------------
    # WAYPOINT MISSION EXECUTOR
    # ------------------------
    def _execute_waypoint_mission(self, mission_data: dict) -> tuple:
        """
        Execute a waypoint-based exploration mission.

        Waypoints are driving-style instructions like "2m north", "turn 45", etc.
        The rover navigates each waypoint sequentially, with obstacle avoidance.
        If detour distance exceeds threshold, mission aborts and returns to dock.

        Args:
            mission_data: Dict with 'waypoints' list and 'return_to_dock' flag

        Returns:
            Tuple of (success: bool, metrics: dict)
        """
        # Try to initialize navigation, fall back to simple nav if unavailable
        if not self.navigator and not self.use_simple_nav:
            try:
                self._initialize_navigation()
            except RuntimeError as e:
                self.get_logger().warn(f"Nav2 unavailable: {e}")
                self.get_logger().info("Using simple cmd_vel navigation (no obstacle avoidance)")
                self.use_simple_nav = True

        # Store dock position for return (may be None if TF not available)
        self.dock_pose = self._get_current_pose_safe()
        self._publish_dock_transform(self.dock_pose)
        dock_x = self.dock_pose.pose.position.x
        dock_y = self.dock_pose.pose.position.y

        # Parse waypoints from mission data
        waypoints_raw = mission_data.get("waypoints", [])
        return_to_dock = mission_data.get("return_to_dock", True)

        self.get_logger().info(
            f"Starting waypoint mission: {len(waypoints_raw)} waypoints, "
            f"return_to_dock={return_to_dock}"
        )

        # Clear costmaps to remove dock obstacle from local costmap
        # This allows the rover to move away from the dock without being blocked
        try:
            self.get_logger().info("Clearing costmaps before navigation...")
            self.navigator.clearAllCostmaps()
            time.sleep(0.5)  # Brief pause for costmap to rebuild with lidar data
        except Exception as e:
            self.get_logger().warn(f"Failed to clear costmaps: {e}")

        # Convert raw waypoint dicts to Waypoint objects
        waypoints = []
        for wp_dict in waypoints_raw:
            wp = Waypoint(
                type=wp_dict.get("type", "move"),
                distance=wp_dict.get("distance", 0.0),
                direction=wp_dict.get("direction", "forward"),
                angle=wp_dict.get("angle", 0.0)
            )
            waypoints.append(wp)

        # Metrics tracking
        metrics = {
            "waypoints_total": len(waypoints),
            "waypoints_completed": 0,
            "waypoints_failed": 0,
            "total_distance_planned": sum(wp.distance for wp in waypoints if wp.type == "move"),
            "total_distance_traveled": 0.0,
            "total_detour_distance": 0.0,
            "aborted": False,
            "abort_reason": "",
            "waypoint_results": []
        }

        # Get current heading from pose orientation
        current_pose = self._get_current_pose()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = self._quaternion_to_yaw(current_pose.pose.orientation)

        # Execute each waypoint
        for idx, waypoint in enumerate(waypoints):
            self.get_logger().info(
                f"Waypoint {idx + 1}/{len(waypoints)}: "
                f"{waypoint.type} {waypoint.distance}m {waypoint.direction} "
                f"(angle: {waypoint.angle}°)"
            )

            result = WaypointResult(
                waypoint_index=idx,
                success=False,
                actual_distance=0.0,
                blocked_at=None,
                detour_distance=0.0,
                reason=""
            )

            if waypoint.type == "turn":
                # Execute turn in place
                if self.use_simple_nav:
                    turn_success = self._simple_turn(waypoint.angle)
                else:
                    turn_success = self._execute_turn(waypoint.angle)
                current_yaw += math.radians(waypoint.angle)
                # Normalize yaw to [-pi, pi]
                while current_yaw > math.pi:
                    current_yaw -= 2 * math.pi
                while current_yaw < -math.pi:
                    current_yaw += 2 * math.pi

                result.success = turn_success
                result.reason = "Turn completed" if turn_success else "Turn failed"
                metrics["waypoint_results"].append(result)

                if turn_success:
                    metrics["waypoints_completed"] += 1
                else:
                    metrics["waypoints_failed"] += 1

            elif waypoint.type == "move":
                # Calculate target position based on direction
                target_x, target_y = self._calculate_target_position(
                    current_x, current_y, current_yaw,
                    waypoint.distance, waypoint.direction
                )

                self.get_logger().info(
                    f"  Target: ({target_x:.2f}, {target_y:.2f}) from "
                    f"current ({current_x:.2f}, {current_y:.2f})"
                )

                # Navigate to target
                if self.use_simple_nav:
                    # Simple cmd_vel navigation (no obstacle avoidance)
                    nav_success = self._simple_move(waypoint.distance, waypoint.direction, current_yaw)
                    actual_dist = waypoint.distance if nav_success else 0.0
                    detour_dist = 0.0
                    blocked_pos = None
                else:
                    # Full Nav2 navigation with obstacle avoidance
                    nav_success, actual_dist, detour_dist, blocked_pos = self._navigate_to_target(
                        target_x, target_y, waypoint.distance
                    )

                result.success = nav_success
                result.actual_distance = actual_dist
                result.detour_distance = detour_dist
                result.blocked_at = blocked_pos

                metrics["total_distance_traveled"] += actual_dist
                metrics["total_detour_distance"] += detour_dist

                if nav_success:
                    metrics["waypoints_completed"] += 1
                    result.reason = "Waypoint reached"
                    # Update current position
                    current_pose = self._get_current_pose()
                    current_x = current_pose.pose.position.x
                    current_y = current_pose.pose.position.y
                    current_yaw = self._quaternion_to_yaw(current_pose.pose.orientation)
                else:
                    metrics["waypoints_failed"] += 1
                    result.reason = "Navigation failed or blocked"

                    # Check if detour is too large - abort mission
                    if detour_dist > WAYPOINT_DETOUR_MULTIPLIER * waypoint.distance:
                        metrics["aborted"] = True
                        metrics["abort_reason"] = (
                            f"Detour too large at waypoint {idx + 1}: "
                            f"{detour_dist:.2f}m detour for {waypoint.distance:.2f}m waypoint "
                            f"(threshold: {WAYPOINT_DETOUR_MULTIPLIER}x)"
                        )
                        self.get_logger().warn(f"ABORTING: {metrics['abort_reason']}")
                        result.reason = metrics["abort_reason"]
                        metrics["waypoint_results"].append(result)
                        break

                metrics["waypoint_results"].append(result)

        # Return to dock if requested (or if aborted)
        if return_to_dock or metrics["aborted"]:
            self.get_logger().info("Returning to dock...")
            if self.use_simple_nav:
                # Simple nav: just log completion (can't navigate back without position tracking)
                self.get_logger().info("(Simple nav mode - skipping return navigation)")
                return_success = True
            else:
                # Pass full dock_pose to preserve spawn orientation for precise re-docking
                return_success = self._return_to_position(dock_x, dock_y, self.dock_pose)

            if return_success:
                self.get_logger().info("✓ Returned to dock successfully")
            else:
                self.get_logger().warn("⚠ Failed to return to dock precisely")

            metrics["returned_to_dock"] = return_success

        # Determine overall success
        success = (
            metrics["waypoints_completed"] == metrics["waypoints_total"]
            and not metrics["aborted"]
        )

        self.get_logger().info(
            f"Waypoint mission complete: {metrics['waypoints_completed']}/{metrics['waypoints_total']} "
            f"waypoints, {metrics['total_distance_traveled']:.2f}m traveled, "
            f"aborted={metrics['aborted']}"
        )

        # Drain battery based on distance traveled
        if self.hw and metrics["total_distance_traveled"] > 0:
            self.hw.drain_battery(metrics["total_distance_traveled"])
            self.get_logger().info(f"Battery: {self.hw.get_battery_percentage()*100:.0f}%")

        return success, metrics

    def _calculate_target_position(
        self, current_x: float, current_y: float, current_yaw: float,
        distance: float, direction: str
    ) -> tuple:
        """
        Calculate target position based on direction instruction.

        Args:
            current_x, current_y: Current robot position
            current_yaw: Current heading in radians
            distance: Distance to travel in meters
            direction: "north", "south", "east", "west", "forward", or "backward"

        Returns:
            Tuple of (target_x, target_y)
        """
        if direction == "north":
            # North is +Y in standard ROS coordinate frame
            return current_x, current_y + distance
        elif direction == "south":
            return current_x, current_y - distance
        elif direction == "east":
            # East is +X
            return current_x + distance, current_y
        elif direction == "west":
            return current_x - distance, current_y
        elif direction == "forward":
            # Forward is along current heading
            target_x = current_x + distance * math.cos(current_yaw)
            target_y = current_y + distance * math.sin(current_yaw)
            return target_x, target_y
        elif direction == "backward":
            # Backward is opposite to current heading
            target_x = current_x - distance * math.cos(current_yaw)
            target_y = current_y - distance * math.sin(current_yaw)
            return target_x, target_y
        else:
            # Default to forward
            self.get_logger().warn(f"Unknown direction '{direction}', using forward")
            target_x = current_x + distance * math.cos(current_yaw)
            target_y = current_y + distance * math.sin(current_yaw)
            return target_x, target_y

    def _quaternion_to_yaw(self, q) -> float:
        """Convert quaternion to yaw angle in radians."""
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _yaw_to_quaternion(self, yaw: float):
        """Convert yaw angle to quaternion."""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _execute_turn(self, angle_degrees: float) -> bool:
        """
        Execute an in-place turn by the specified angle.

        Args:
            angle_degrees: Angle to turn (positive = clockwise)

        Returns:
            True if turn completed successfully
        """
        try:
            # Get current pose
            current_pose = self._get_current_pose()
            current_yaw = self._quaternion_to_yaw(current_pose.pose.orientation)

            # Calculate target yaw (negative because positive degrees = clockwise = negative radians)
            target_yaw = current_yaw - math.radians(angle_degrees)

            # Normalize to [-pi, pi]
            while target_yaw > math.pi:
                target_yaw -= 2 * math.pi
            while target_yaw < -math.pi:
                target_yaw += 2 * math.pi

            # Create target pose at current position with new orientation
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position = current_pose.pose.position
            target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

            # Use Nav2 to rotate (it handles in-place rotation)
            self.navigator.goToPose(target_pose)

            # Wait for completion
            start_time = time.time()
            while not self.navigator.isTaskComplete():
                if time.time() - start_time > WAYPOINT_NAV_TIMEOUT:
                    self.get_logger().warn("Turn timeout")
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.1)

            result = self.navigator.getResult()
            return result.value == 1  # SUCCEEDED = 1

        except Exception as e:
            self.get_logger().error(f"Turn execution failed: {e}")
            return False

    def _navigate_to_target(
        self, target_x: float, target_y: float, expected_distance: float
    ) -> tuple:
        """
        Navigate to target position with obstacle avoidance.

        Args:
            target_x, target_y: Target position
            expected_distance: Expected straight-line distance

        Returns:
            Tuple of (success, actual_distance, detour_distance, blocked_position)
            - success: True if reached target
            - actual_distance: Distance actually traveled
            - detour_distance: Extra distance beyond straight line
            - blocked_position: (x, y) if blocked, None otherwise
        """
        try:
            # Get starting position
            start_pose = self._get_current_pose()
            start_x = start_pose.pose.position.x
            start_y = start_pose.pose.position.y

            # Create target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = target_x
            target_pose.pose.position.y = target_y
            target_pose.pose.position.z = 0.0

            # Calculate orientation towards target
            dx = target_x - start_x
            dy = target_y - start_y
            target_yaw = math.atan2(dy, dx)
            target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

            # Start navigation
            self.get_logger().info(f"Sending goal to Nav2: ({target_x:.2f}, {target_y:.2f})")
            self.navigator.goToPose(target_pose)

            # Track distance while navigating
            last_x, last_y = start_x, start_y
            actual_distance = 0.0
            loop_count = 0

            start_time = time.time()
            while not self.navigator.isTaskComplete():
                loop_count += 1
                elapsed = time.time() - start_time

                if elapsed > WAYPOINT_NAV_TIMEOUT:
                    self.get_logger().warn(f"Navigation timeout after {elapsed:.1f}s")
                    self.navigator.cancelTask()

                    # Get final position
                    final_pose = self._get_current_pose()
                    blocked_pos = (final_pose.pose.position.x, final_pose.pose.position.y)
                    detour = actual_distance - expected_distance
                    return False, actual_distance, max(0, detour), blocked_pos

                # Update distance tracking
                current_pose = self._get_current_pose()
                curr_x = current_pose.pose.position.x
                curr_y = current_pose.pose.position.y

                step_dist = math.sqrt((curr_x - last_x)**2 + (curr_y - last_y)**2)
                actual_distance += step_dist
                last_x, last_y = curr_x, curr_y

                # Log progress every 5 iterations (1 second)
                if loop_count % 5 == 0:
                    dist_remaining = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)
                    self.get_logger().info(
                        f"  Nav progress: pos=({curr_x:.2f}, {curr_y:.2f}), "
                        f"dist_remaining={dist_remaining:.2f}m, elapsed={elapsed:.1f}s"
                    )

                time.sleep(0.2)

            # Check result
            result = self.navigator.getResult()
            elapsed = time.time() - start_time

            # Get final position
            final_pose = self._get_current_pose()
            final_x = final_pose.pose.position.x
            final_y = final_pose.pose.position.y

            # Check if we actually reached the target
            dist_to_target = math.sqrt((final_x - target_x)**2 + (final_y - target_y)**2)

            self.get_logger().info(
                f"Nav completed: result={result}, loops={loop_count}, "
                f"final=({final_x:.2f}, {final_y:.2f}), dist_to_target={dist_to_target:.2f}m, "
                f"elapsed={elapsed:.1f}s"
            )

            if result.value == 1 and dist_to_target < WAYPOINT_POSITION_TOLERANCE:
                # Success
                detour = actual_distance - expected_distance
                return True, actual_distance, max(0, detour), None
            else:
                # Failed or didn't reach target
                self.get_logger().warn(
                    f"Navigation failed: result={result.value}, dist_to_target={dist_to_target:.2f}m"
                )
                blocked_pos = (final_x, final_y)
                detour = actual_distance - expected_distance
                return False, actual_distance, max(0, detour), blocked_pos

        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
            return False, 0.0, 0.0, None

    def _return_to_position(self, x: float, y: float, dock_pose: Optional[PoseStamped] = None) -> bool:
        """
        Navigate back to a specific position (usually dock).

        Args:
            x, y: Target position
            dock_pose: Optional full dock pose with orientation for precise re-docking

        Returns:
            True if reached position successfully
        """
        try:
            # Use full dock pose if provided (preserves spawn orientation)
            if dock_pose is not None:
                target_pose = PoseStamped()
                target_pose.header.frame_id = 'map'
                target_pose.header.stamp = self.get_clock().now().to_msg()
                target_pose.pose = dock_pose.pose
                self.get_logger().info(
                    f"Returning to dock with original orientation (preserving spawn pose)"
                )
            else:
                # Fallback: calculate orientation towards target
                current_pose = self._get_current_pose()
                curr_x = current_pose.pose.position.x
                curr_y = current_pose.pose.position.y

                target_pose = PoseStamped()
                target_pose.header.frame_id = 'map'
                target_pose.header.stamp = self.get_clock().now().to_msg()
                target_pose.pose.position.x = x
                target_pose.pose.position.y = y
                target_pose.pose.position.z = 0.0

                dx = x - curr_x
                dy = y - curr_y
                target_yaw = math.atan2(dy, dx)
                target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

            # Navigate
            self.navigator.goToPose(target_pose)

            # Wait for completion with extended timeout for return journey
            start_time = time.time()
            return_timeout = WAYPOINT_NAV_TIMEOUT * 3  # Allow more time for return

            while not self.navigator.isTaskComplete():
                if time.time() - start_time > return_timeout:
                    self.get_logger().warn("Return to dock timeout")
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.2)

            result = self.navigator.getResult()

            # Check if we reached the dock
            final_pose = self._get_current_pose()
            dist_to_dock = math.sqrt(
                (final_pose.pose.position.x - x)**2 +
                (final_pose.pose.position.y - y)**2
            )

            return result.value == 1 and dist_to_dock < WAYPOINT_POSITION_TOLERANCE * 2

        except Exception as e:
            self.get_logger().error(f"Return to position failed: {e}")
            return False

    # ------------------------
    # COVERAGE MISSION EXECUTOR
    # ------------------------
    def _execute_coverage_mission(self, goal: CoverageGoal) -> tuple:
        """
        Execute autonomous coverage exploration mission.

        Uses frontier-based exploration with battery monitoring.
        Rover explores until one of these conditions:
        - Target coverage achieved
        - Battery below threshold
        - Time limit exceeded
        - No more frontiers found

        Args:
            goal: CoverageGoal with target coverage, sector bounds, time limit, etc.

        Returns:
            Tuple of (success: bool, metrics: dict)
        """
        self.get_logger().info(
            f"Starting coverage mission: target={goal.target_coverage:.0%}, "
            f"sector={goal.sector or 'ALL'}, max_time={goal.max_exploration_time}s"
        )

        # Initialize tracking
        self._coverage_mission_start_time = time.time()
        self._coverage_distance_traveled = 0.0
        last_pose = self._get_current_pose()
        return_reason = ""

        # Metrics to return
        metrics = {
            "coverage": 0.0,
            "distance": 0.0,
            "frontiers_explored": 0,
            "battery_start": self.hw.get_battery_percentage() if self.hw else 1.0,
            "battery_end": 0.0,
            "return_reason": "",
            "sector": goal.sector or "ALL",
        }

        # Start periodic status updates (every 5 seconds)
        self._start_coverage_status_updates(goal)

        try:
            frontier_failures = 0
            max_frontier_failures = self.get_parameter('exploration.no_frontier_limit').value

            while True:
                # Check termination conditions
                elapsed = time.time() - self._coverage_mission_start_time

                # 1. Time limit
                if elapsed >= goal.max_exploration_time:
                    return_reason = "timeout"
                    self.get_logger().info(f"Coverage mission timeout after {elapsed:.0f}s")
                    break

                # 2. Battery threshold
                if goal.return_on_low_battery and self.hw:
                    battery = self.hw.get_battery_percentage()
                    if battery < goal.battery_return_threshold:
                        return_reason = "low_battery"
                        self.get_logger().info(
                            f"Low battery ({battery:.0%}) - returning to dock"
                        )
                        break

                # 3. Coverage target (check from explorer)
                if self.explorer:
                    current_coverage = self.explorer.get_coverage()
                    metrics["coverage"] = current_coverage
                    if current_coverage >= goal.target_coverage:
                        return_reason = "coverage_achieved"
                        self.get_logger().info(
                            f"Coverage target achieved: {current_coverage:.1%}"
                        )
                        break

                # 4. Find next frontier (with optional sector bounds)
                frontier = self._find_frontier_in_sector(goal.sector_bounds)

                if frontier is None:
                    frontier_failures += 1
                    self.get_logger().info(
                        f"No frontier found ({frontier_failures}/{max_frontier_failures})"
                    )
                    if frontier_failures >= max_frontier_failures:
                        return_reason = "no_frontiers"
                        self.get_logger().info("No more frontiers - exploration complete")
                        break
                    time.sleep(1.0)  # Brief pause before retry
                    continue

                frontier_failures = 0  # Reset on success

                # Navigate to frontier with distance tracking
                nav_success, distance = self._navigate_to_frontier(frontier)
                self._coverage_distance_traveled += distance
                metrics["distance"] = self._coverage_distance_traveled

                # Drain battery based on distance
                if self.hw and distance > 0:
                    self.hw.drain_battery(distance)

                # Update pose for next iteration
                current_pose = self._get_current_pose()
                last_pose = current_pose

                if nav_success:
                    metrics["frontiers_explored"] += 1
                else:
                    self.get_logger().warn("Failed to reach frontier, trying next")

        finally:
            # Stop status updates
            self._stop_coverage_status_updates()

        # Record final metrics
        metrics["battery_end"] = self.hw.get_battery_percentage() if self.hw else 1.0
        metrics["return_reason"] = return_reason

        # Publish final status before returning
        self._publish_coverage_status(
            metrics["coverage"],
            metrics["battery_end"],
            returning=True,
            reason=return_reason
        )

        # Return to dock
        if self.dock_pose:
            self.get_logger().info("Returning to dock...")
            dock_x = self.dock_pose.pose.position.x
            dock_y = self.dock_pose.pose.position.y
            return_success = self._return_to_position(dock_x, dock_y, self.dock_pose)

            if return_success and self.hw:
                # Recharge battery when docked
                self.hw.charge_battery(0.5)  # Partial recharge
                self.get_logger().info(
                    f"Docked. Battery recharged to {self.hw.get_battery_percentage():.0%}"
                )

        success = return_reason in ["coverage_achieved", "timeout", "no_frontiers"]
        return success, metrics

    def _find_frontier_in_sector(self, sector_bounds: tuple = None):
        """
        Find the best frontier, optionally filtered to sector bounds.

        Uses information gain / distance scoring per best practices.

        Args:
            sector_bounds: Optional (x_min, y_min, x_max, y_max) to filter frontiers

        Returns:
            Best frontier point or None if no frontiers found
        """
        if not self.explorer:
            return None

        # Get all frontiers from explorer
        frontiers = self.explorer.find_frontiers()
        if not frontiers:
            return None

        # Filter to sector if specified
        if sector_bounds:
            x_min, y_min, x_max, y_max = sector_bounds
            frontiers = [
                f for f in frontiers
                if x_min <= f.x <= x_max and y_min <= f.y <= y_max
            ]
            if not frontiers:
                self.get_logger().debug(f"No frontiers in sector {sector_bounds}")
                return None

        # Score frontiers by info gain / distance (best practice)
        robot_pose = self._get_current_pose()
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y

        scored = []
        for f in frontiers:
            dist = math.sqrt((f.x - robot_x)**2 + (f.y - robot_y)**2)
            if dist < 0.1:
                dist = 0.1  # Avoid division by zero

            # Info gain approximated by frontier size (already filtered by explorer)
            info_gain = getattr(f, 'size', 10)  # Default size if not available
            score = info_gain / dist

            scored.append((f, score))

        # Sort by score (highest first)
        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[0][0]

    def _navigate_to_frontier(self, frontier) -> tuple:
        """
        Navigate to a frontier point with distance tracking.

        Args:
            frontier: Frontier point with x, y coordinates

        Returns:
            Tuple of (success: bool, distance_traveled: float)
        """
        start_pose = self._get_current_pose()
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y

        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = frontier.x
        target_pose.pose.position.y = frontier.y
        target_pose.pose.position.z = 0.0

        # Calculate orientation towards target
        dx = frontier.x - start_x
        dy = frontier.y - start_y
        target_yaw = math.atan2(dy, dx)
        target_pose.pose.orientation = self._yaw_to_quaternion(target_yaw)

        try:
            self.navigator.goToPose(target_pose)

            # Wait for completion with timeout
            nav_timeout = self.get_parameter('exploration.nav_timeout').value
            start_time = time.time()

            while not self.navigator.isTaskComplete():
                if time.time() - start_time > nav_timeout:
                    self.get_logger().warn("Navigation timeout")
                    self.navigator.cancelTask()
                    break
                time.sleep(0.1)

            # Calculate distance traveled
            end_pose = self._get_current_pose()
            end_x = end_pose.pose.position.x
            end_y = end_pose.pose.position.y
            distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)

            result = self.navigator.getResult()
            success = result.value == 1  # SUCCEEDED

            return success, distance

        except Exception as e:
            self.get_logger().error(f"Frontier navigation failed: {e}")
            return False, 0.0

    def _start_coverage_status_updates(self, goal: CoverageGoal):
        """Start periodic coverage status publishing."""
        if self._coverage_status_timer:
            self._coverage_status_timer.cancel()

        # Publish every 5 seconds
        interval = 5.0
        self._coverage_status_timer = self.create_timer(
            interval,
            lambda: self._publish_coverage_status_periodic(goal)
        )

    def _stop_coverage_status_updates(self):
        """Stop periodic coverage status publishing."""
        if self._coverage_status_timer:
            self._coverage_status_timer.cancel()
            self._coverage_status_timer = None

    def _publish_coverage_status_periodic(self, goal: CoverageGoal):
        """Periodic callback to publish coverage status."""
        coverage = self.explorer.get_coverage() if self.explorer else 0.0
        battery = self.hw.get_battery_percentage() if self.hw else 1.0
        self._publish_coverage_status(coverage, battery, returning=False, reason="exploring")

    def _publish_coverage_status(
        self,
        coverage: float,
        battery: float,
        returning: bool = False,
        reason: str = ""
    ):
        """Publish coverage status update to dock."""
        frontiers = len(self.explorer.find_frontiers()) if self.explorer else 0

        status = CoverageStatus(
            module_id=self.module_id,
            current_coverage=coverage,
            battery_remaining=battery,
            distance_traveled=self._coverage_distance_traveled,
            frontiers_remaining=frontiers,
            returning_to_dock=returning,
            reason=reason
        )

        self.pub_coverage_status.publish(
            String(data=common.coverage_status_encode(status))
        )

        self.get_logger().debug(
            f"Coverage status: {coverage:.1%} coverage, {battery:.0%} battery, "
            f"{frontiers} frontiers, returning={returning}"
        )

    # ------------------------
    # ACTION SERVER CALLBACKS
    # ------------------------
    def _goal_callback(self, goal_request):
        """Accept or reject goal requests."""
        # Check if module is in NORMAL state and ready to accept tasks
        if self.state != ModuleState.NORMAL:
            self.get_logger().warn(
                f"Rejecting task goal: module not in NORMAL state (current: {self.state})"
            )
            return GoalResponse.REJECT

        # Check if module_id matches
        if goal_request.module_id != self.module_id:
            self.get_logger().warn(
                f"Rejecting task goal: module_id mismatch "
                f"(requested: {goal_request.module_id}, actual: {self.module_id})"
            )
            return GoalResponse.REJECT

        self.get_logger().info(f"Accepting task goal: {goal_request.task}")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info("Received cancel request for task")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execute_action_callback(self, goal_handle: ServerGoalHandle):
        """Execute task via action server (replaces topic-based execute_task)."""
        self.get_logger().info(f"Executing task via action server: {goal_handle.request.task}")

        self._current_goal_handle = goal_handle
        self._cancel_requested = False

        # Stop heartbeat and transition to FIELD_OPS
        self.stop_heartbeat()
        self.state = ModuleState.FIELD_OPS
        self._start_task_watchdog(goal_handle.request.task)

        task_name = goal_handle.request.task
        success = False
        map_data_b64 = ""
        map_yaml_b64 = ""
        metrics = {}
        start_time = time.time()

        try:
            # Send initial feedback
            feedback = ExecuteTask.Feedback()
            feedback.status = "starting"
            feedback.progress = 0.0
            feedback.elapsed_time = 0.0
            goal_handle.publish_feedback(feedback)

            if "explore" in task_name.lower():
                # Initialize navigation if not already done
                if not self.navigator:
                    feedback.status = "initializing_nav2"
                    goal_handle.publish_feedback(feedback)

                    try:
                        self._initialize_navigation()
                    except RuntimeError as e:
                        self.get_logger().error(f"Navigation initialization failed: {e}")
                        result = ExecuteTask.Result()
                        result.success = False
                        result.note = "Nav2 not available - run in full mode (-#f) for exploration"
                        result.duration = time.time() - start_time
                        self._cleanup_after_task()
                        goal_handle.abort()
                        return result

                # Check for cancellation
                if self._cancel_requested or not goal_handle.is_active:
                    self.get_logger().info("Task cancelled during initialization")
                    result = ExecuteTask.Result()
                    result.success = False
                    result.note = "Task cancelled"
                    result.duration = time.time() - start_time
                    self._cleanup_after_task()
                    goal_handle.canceled()
                    return result

                # Store dock position
                self.dock_pose = self._get_current_pose()
                self._publish_dock_transform(self.dock_pose)

                # Send exploration feedback
                feedback.status = "exploring"
                feedback.progress = 0.1
                feedback.elapsed_time = time.time() - start_time
                goal_handle.publish_feedback(feedback)

                # Execute exploration
                explore_timeout = self.get_parameter('exploration.timeout').value
                success, metrics = self.explorer.explore(
                    duration=explore_timeout,
                    feedback_callback=lambda coverage, frontiers: self._publish_exploration_feedback(
                        goal_handle, coverage, frontiers, time.time() - start_time
                    )
                )

                # Check for cancellation before return
                if self._cancel_requested or not goal_handle.is_active:
                    self.get_logger().info("Task cancelled during exploration")
                    result = ExecuteTask.Result()
                    result.success = False
                    result.note = "Task cancelled"
                    result.duration = time.time() - start_time
                    self._cleanup_after_task()
                    goal_handle.canceled()
                    return result

                # Return to dock
                if success and self.dock_pose:
                    feedback.status = "returning_to_dock"
                    feedback.progress = 0.9
                    goal_handle.publish_feedback(feedback)

                    return_success = self.explorer.return_to_dock(self.dock_pose)
                    if not return_success:
                        self.get_logger().warn("Failed to return to dock, using best effort")

                # Save map
                map_data_b64, map_yaml_b64 = self._save_and_serialize_map()

            else:
                # Fallback for non-exploration tasks
                delay_time = self.get_parameter('exploration.timeout').value
                self.get_logger().info(f"Unknown task type '{task_name}', using delay simulation ({delay_time}s)")
                time.sleep(delay_time)
                success = True

        except Exception as e:
            self.get_logger().error(f"Task execution failed: {e}")
            success = False

        finally:
            self._cleanup_after_task()

        # Prepare result
        result = ExecuteTask.Result()
        result.success = success
        result.note = f"Exploration complete: {metrics.get('coverage', 0):.1%} coverage" if success else "Task failed"
        result.map_data = map_data_b64
        result.map_yaml = map_yaml_b64
        result.coverage = metrics.get('coverage', 0.0)
        result.duration = time.time() - start_time
        result.distance_traveled = metrics.get('distance', 0.0)
        result.frontiers_explored = metrics.get('frontiers', 0)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(f"Action task {task_name} complete")
        return result

    def _publish_exploration_feedback(self, goal_handle, coverage, frontiers, elapsed):
        """Publish feedback during exploration."""
        if goal_handle.is_active:
            feedback = ExecuteTask.Feedback()
            feedback.status = "exploring"
            feedback.progress = min(coverage, 0.9)  # Cap at 90% until return to dock
            feedback.current_coverage = coverage
            feedback.frontiers_found = frontiers
            feedback.elapsed_time = elapsed
            goal_handle.publish_feedback(feedback)

    def _cleanup_after_task(self):
        """Clean up after task execution (for action server)."""
        self._stop_task_watchdog()
        self.state = ModuleState.NORMAL
        self.start_heartbeat()
        self.last_task_complete_time = time.time()  # Update for bid cost calculation
        self._bid_calculator.mark_task_complete()  # Sync with bid calculator
        self._current_goal_handle = None
        self._cancel_requested = False

    def _initialize_navigation(self, max_retries=3):
        """
        Initialize Nav2 navigator and explorer with retry logic.

        Args:
            max_retries: Maximum number of initialization attempts

        Raises:
            RuntimeError: If initialization fails after all retries
        """
        last_error = None

        for attempt in range(max_retries):
            try:
                self.get_logger().info(
                    f"Initializing navigation components for {self.robot_namespace}... "
                    f"(attempt {attempt + 1}/{max_retries})"
                )

                # Create BasicNavigator with unique node name and namespace
                nav_node_name = f'navigator_{self.module_id.replace("-", "_")}'

                # BasicNavigator needs the namespace to communicate with the correct robot
                if self.robot_namespace:
                    self.navigator = BasicNavigator(node_name=nav_node_name, namespace=self.robot_namespace)
                else:
                    self.navigator = BasicNavigator(node_name=nav_node_name)

                # Add navigator node to executor so it can be spun
                if self.executor:
                    self.executor.add_node(self.navigator)

                # Wait for Nav2 action server to be ready using event-driven approach
                # BasicNavigator already has the action client for navigate_to_pose
                # With namespace, this becomes /{namespace}/navigate_to_pose
                # We just need to wait for the server to become available
                action_server_name = f"/{self.robot_namespace}/navigate_to_pose" if self.robot_namespace else "/navigate_to_pose"
                self.get_logger().info(f"Waiting for Nav2 action server ({action_server_name})...")

                # Use the navigator's built-in action client's wait_for_server method
                # This is event-driven - returns as soon as server is detected
                if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=60.0):
                    raise RuntimeError(
                        "Nav2 action server not available after 60s. "
                        "Are you running in full mode (-#f)?"
                    )

                self.get_logger().info("✓ Nav2 action server ready")

                # Note: We skip waitUntilNav2Active() because our Nav2 nodes are launched
                # with autostart:true and don't expose lifecycle get_state services.
                # The action server being ready (above) is sufficient for navigation.
                # Give Nav2 a moment to fully initialize after action server is up
                time.sleep(1.0)
                self.get_logger().info("✓ Nav2 ready for navigation")

                # Create Explorer with robot namespace for TF frame resolution
                self.explorer = Explorer(self, self.navigator, self.robot_namespace)

                self.get_logger().info("Navigation components ready")
                return  # Success!

            except Exception as e:
                last_error = e
                self.get_logger().warn(
                    f"Navigation initialization attempt {attempt + 1} failed: {e}"
                )

                # Clean up on failure
                if self.navigator and self.executor:
                    try:
                        self.executor.remove_node(self.navigator)
                    except Exception:
                        pass
                self.navigator = None
                self.explorer = None

                if attempt < max_retries - 1:
                    retry_delay = 2.0 * (attempt + 1)  # Exponential backoff
                    self.get_logger().info(f"Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)

        # All retries failed
        raise RuntimeError(
            f"Navigation initialization failed after {max_retries} attempts. "
            f"Last error: {last_error}"
        )

    def _publish_dock_transform(self, dock_pose: PoseStamped):
        """Publish a static transform for the dock location."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = f'{self.module_id}_dock'

        transform.transform.translation.x = dock_pose.pose.position.x
        transform.transform.translation.y = dock_pose.pose.position.y
        transform.transform.translation.z = dock_pose.pose.position.z
        transform.transform.rotation = dock_pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Published dock transform at ({dock_pose.pose.position.x:.2f}, {dock_pose.pose.position.y:.2f})")

    def _get_current_pose(self) -> PoseStamped:
        """Get the current robot pose using TF2."""
        try:
            # Use the explorer's TF buffer if available (it's connected to the
            # namespaced navigator, so it can see /robot_1/tf). Fall back to
            # self.tf_buffer for non-namespaced operation.
            tf_buffer = self.explorer.tf_buffer if self.explorer else self.tf_buffer

            # Try to get transform from map to base_link
            # This gives us the robot's pose in the map frame
            # NOTE: For multi-robot setups, frames ARE namespaced (e.g., Akko/base_link)
            base_frame = f'{self.robot_namespace}/base_link' if self.robot_namespace else 'base_link'
            transform = tf_buffer.lookup_transform(
                'map',
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Convert transform to PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get robot pose via TF2: {e}")
            self.get_logger().warn("Falling back to origin pose")

            # Fallback: return origin pose
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            return pose

    def _get_current_pose_safe(self) -> PoseStamped:
        """Get current pose, returning origin if unavailable (for simple nav mode)."""
        try:
            return self._get_current_pose()
        except Exception:
            # Return origin pose for simple nav mode
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.orientation.w = 1.0
            return pose

    def get_position(self) -> tuple[float, float]:
        """
        Get current (x, y) position for PoseProvider protocol.

        Used by BidCalculator for dock obstruction penalty calculation.
        """
        pose = self._get_current_pose_safe()
        return (pose.pose.position.x, pose.pose.position.y)

    # -------------------------
    # Simple Navigation (cmd_vel fallback when Nav2 unavailable)
    # -------------------------
    def _simple_move(self, distance: float, direction: str, current_yaw: float) -> bool:
        """
        Execute a simple move using cmd_vel (no obstacle avoidance).

        Args:
            distance: Distance to travel in meters
            direction: 'north', 'south', 'east', 'west', or 'forward'
            current_yaw: Current heading in radians

        Returns:
            True if move completed
        """
        # Calculate velocity components based on direction
        LINEAR_SPEED = 0.3  # m/s

        if direction == "forward":
            # Move in current heading direction
            vx = LINEAR_SPEED * math.cos(current_yaw)
            vy = LINEAR_SPEED * math.sin(current_yaw)
        elif direction == "north":
            vx = 0.0
            vy = LINEAR_SPEED
        elif direction == "south":
            vx = 0.0
            vy = -LINEAR_SPEED
        elif direction == "east":
            vx = LINEAR_SPEED
            vy = 0.0
        elif direction == "west":
            vx = -LINEAR_SPEED
            vy = 0.0
        else:
            self.get_logger().warn(f"Unknown direction: {direction}")
            return False

        # Calculate duration based on distance
        duration = distance / LINEAR_SPEED

        self.get_logger().info(f"Simple move: {distance}m {direction} (duration: {duration:.1f}s)")

        # Send velocity commands for the calculated duration
        twist = Twist()
        twist.linear.x = LINEAR_SPEED  # Always move forward in robot frame
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        start_time = time.time()
        rate = 10  # Hz
        while time.time() - start_time < duration:
            self.pub_cmd_vel.publish(twist)
            time.sleep(1.0 / rate)

        # Stop
        twist.linear.x = 0.0
        self.pub_cmd_vel.publish(twist)

        return True

    def _simple_turn(self, angle_degrees: float) -> bool:
        """
        Execute a simple turn using cmd_vel.

        Args:
            angle_degrees: Angle to turn (positive = counter-clockwise)

        Returns:
            True if turn completed
        """
        ANGULAR_SPEED = 0.5  # rad/s

        angle_rad = math.radians(angle_degrees)
        duration = abs(angle_rad) / ANGULAR_SPEED

        self.get_logger().info(f"Simple turn: {angle_degrees}° (duration: {duration:.1f}s)")

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = ANGULAR_SPEED if angle_rad > 0 else -ANGULAR_SPEED

        start_time = time.time()
        rate = 10  # Hz
        while time.time() - start_time < duration:
            self.pub_cmd_vel.publish(twist)
            time.sleep(1.0 / rate)

        # Stop
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

        return True

    def _save_and_serialize_map(self) -> tuple:
        """
        Save the current SLAM map and serialize it for transmission.

        Returns:
            Tuple of (map_data_b64, map_yaml_b64) - Base64 encoded strings
        """
        try:
            # Create temporary directory for map files (using parameter)
            map_dir = os.path.join(self.map_storage_dir, self.module_id)
            os.makedirs(map_dir, exist_ok=True)

            map_file = f"{map_dir}/exploration_map"

            # Call map_saver_cli to save the map
            self.get_logger().info(f"Saving map to {map_file}...")

            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", map_file, "--ros-args", "-p", "save_map_timeout:=5000.0"],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode != 0:
                self.get_logger().warn(f"Map saver failed: {result.stderr}")
                return "", ""

            # Read and encode the PGM file
            pgm_file = f"{map_file}.pgm"
            yaml_file = f"{map_file}.yaml"

            if not os.path.exists(pgm_file) or not os.path.exists(yaml_file):
                self.get_logger().warn("Map files not found after saving")
                return "", ""

            # Read and compress PGM
            with open(pgm_file, 'rb') as f:
                pgm_data = f.read()
                pgm_compressed = gzip.compress(pgm_data)
                pgm_b64 = base64.b64encode(pgm_compressed).decode('ascii')

            # Read and compress YAML
            with open(yaml_file, 'rb') as f:
                yaml_data = f.read()
                yaml_compressed = gzip.compress(yaml_data)
                yaml_b64 = base64.b64encode(yaml_compressed).decode('ascii')

            self.get_logger().info(
                f"Map serialized: PGM={len(pgm_b64)} bytes (b64), "
                f"YAML={len(yaml_b64)} bytes (b64)"
            )

            return pgm_b64, yaml_b64

        except Exception as e:
            self.get_logger().error(f"Failed to save/serialize map: {e}")
            return "", ""

# ------------------------
# --- Main ---
# ------------------------
def main():
    rclpy.init()

    # Use MultiThreadedExecutor to handle both Module and BasicNavigator nodes
    executor = MultiThreadedExecutor(num_threads=4)

    # Create a temporary node to read parameters
    temp_node = Node('temp_param_reader')
    temp_node.declare_parameter('robot_namespace', '')
    temp_node.declare_parameter('module_id', '')
    temp_node.declare_parameter('spawn_x', 0.0)
    temp_node.declare_parameter('spawn_y', 0.0)

    robot_namespace = temp_node.get_parameter('robot_namespace').value
    module_id_param = temp_node.get_parameter('module_id').value
    spawn_x = temp_node.get_parameter('spawn_x').value
    spawn_y = temp_node.get_parameter('spawn_y').value
    temp_node.destroy_node()

    # Create module node with executor reference and robot namespace
    # Use provided module_id or None to trigger auto-generation
    module_id = module_id_param if module_id_param else None
    node = Module(module_id=module_id, executor=executor, robot_namespace=robot_namespace)

    # Store spawn position for potential use (e.g., Gazebo spawning)
    node.spawn_x = spawn_x
    node.spawn_y = spawn_y

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass  # Graceful shutdown on Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()