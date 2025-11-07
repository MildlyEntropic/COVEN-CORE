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

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# --- Navigation ---
from nav2_simple_commander.robot_navigator import BasicNavigator

# --- COVEN action interfaces ---
from coven_interfaces.action import ExecuteTask

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import ModuleState
from coven_core.exploration import Explorer


# ------------------------
# --- Constants ---
# ------------------------
# NOTE: These are now ROS2 parameters - defaults defined here for reference
# Actual values loaded from config/coven_params.yaml
DEFAULT_HB_PERIOD = 0.8  # seconds between heartbeats
DEFAULT_TASK_TIMEOUT = 300.0  # seconds - watchdog timer for task execution
DEFAULT_MAP_STORAGE_DIR = '~/coven_maps'  # temporary map storage


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

        self.module_id = module_id or f"RR-{str(uuid.uuid4())[:6]}"
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

        # ROS Topics
        self.sub_ident_req = self.create_subscription(String, 'coven/identify_req', self.on_ident_req, 10)
        self.pub_ident_rep = self.create_publisher(String, 'coven/identify_rep', 10)

        self.sub_verify_req = self.create_subscription(String, 'coven/verify_req', self.on_verify_req, 10)
        self.pub_verify_rep = self.create_publisher(String, 'coven/verify_rep', 10)

        self.sub_enable_12v = self.create_subscription(String, 'coven/enable_12v', self.on_power, 10)

        self.pub_hb = self.create_publisher(String, 'coven/heartbeat', 10)

        self.sub_task_req = self.create_subscription(String, 'coven/task_req', self.on_task_req, 10)
        self.pub_task_ack = self.create_publisher(String, 'coven/task_ack', 10)
        self.pub_task_start = self.create_publisher(String, 'coven/task_start', 10)
        self.pub_task_complete = self.create_publisher(String, 'coven/task_complete', 10)

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

    # ------------------------
    # IDENTIFY / VERIFY
    # ------------------------
    def on_ident_req(self, msg: String):
        if self.state != ModuleState.BOOT:
            return
        req = common.ident_req_decode(msg)
        if not req:
            return
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
        # Check for lidar data (scan topic)
        scan_topic = f'{self.robot_namespace}/scan' if self.robot_namespace else '/scan'
        try:
            # Simple check: does topic exist?
            topic_list = self.get_topic_names_and_types()
            lidar_present = any(scan_topic in topic[0] for topic in topic_list)
            self.health_status['lidar'] = lidar_present

            if not lidar_present:
                return False, f"Lidar offline - {scan_topic} topic not found"

        except Exception as e:
            self.get_logger().warn(f"Health check failed: {e}")
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
            if "explore" in task_name.lower():
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
                # BasicNavigator already has the action client for /navigate_to_pose
                # We just need to wait for the server to become available

                self.get_logger().info("Waiting for Nav2 action server (/navigate_to_pose)...")

                # Use the navigator's built-in action client's wait_for_server method
                # This is event-driven - returns as soon as server is detected
                if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=60.0):
                    raise RuntimeError(
                        "Nav2 action server not available after 60s. "
                        "Are you running in full mode (-#f)?"
                    )

                self.get_logger().info("✓ Nav2 action server ready")

                # Create Explorer
                self.explorer = Explorer(self, self.navigator)

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
            # Try to get transform from map to base_link
            # This gives us the robot's pose in the map frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{self.robot_namespace}/base_link' if self.robot_namespace else 'base_link',
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