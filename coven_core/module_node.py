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
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# --- Navigation ---
from nav2_simple_commander.robot_navigator import BasicNavigator

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import ModuleState
from coven_core.exploration import Explorer


# ------------------------
# --- Constants ---
# ------------------------
HB_PERIOD = 0.8  # seconds between heartbeats
TASK_DELAY = 5.0  # simulate time away in field ops


# ------------------------
# --- Module Node ---
# ------------------------
class Module(Node):
    """Module node that manages the FSM lifecycle for a single module."""

    def __init__(self, module_id=None, module_type="ReconRover", fw="0.0.1"):
        super().__init__('coven_module')

        self.module_id = module_id or f"RR-{str(uuid.uuid4())[:6]}"
        self.module_type = module_type
        self.fw = fw
        self.state = ModuleState.BOOT
        self.seq = 0
        self.hb_timer = None

        # Navigation components (initialized on demand)
        self.navigator = None
        self.explorer = None
        self.dock_pose = None  # Store initial pose for return

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
        self.get_logger().info("VERIFY_REQ matched → replying OK")
        rep = common.VerifyRep(module_id=self.module_id, ok=True, reason="")
        self.pub_verify_rep.publish(String(data=common.verify_rep_encode(rep)))

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
        self.hb_timer = self.create_timer(HB_PERIOD, self.send_heartbeat)
        self.get_logger().info(f"Heartbeat started for {self.module_id}")

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

                # Store dock position for return
                self.dock_pose = self._get_current_pose()

                # Execute exploration
                success, metrics = self.explorer.explore(duration=TASK_DELAY)

                # Return to dock
                if success and self.dock_pose:
                    return_success = self.explorer.return_to_dock(self.dock_pose)
                    if not return_success:
                        self.get_logger().warn("Failed to return to dock, using best effort")

                # Save and serialize map
                map_data_b64, map_yaml_b64 = self._save_and_serialize_map()

            else:
                # Fallback to simple delay for other task types
                self.get_logger().info(f"Unknown task type '{task_name}', using delay simulation")
                time.sleep(TASK_DELAY)
                success = True

        except Exception as e:
            self.get_logger().error(f"Task execution failed: {e}")
            success = False

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

    def _initialize_navigation(self):
        """Initialize Nav2 navigator and explorer."""
        self.get_logger().info("Initializing navigation components...")

        # Create BasicNavigator
        self.navigator = BasicNavigator()

        # Wait for Nav2 to be ready with timeout
        self.get_logger().info("Waiting for Nav2 to activate (timeout: 10s)...")
        start_time = time.time()
        nav2_ready = False

        while time.time() - start_time < 10.0:
            try:
                # Check if Nav2 services are available
                service_list = self.navigator.get_service_names_and_types()
                nav2_services = [s for s in service_list if 'nav2' in s[0] or 'navigate_to_pose' in s[0]]

                if nav2_services:
                    nav2_ready = True
                    break

            except Exception as e:
                self.get_logger().debug(f"Nav2 check failed: {e}")

            time.sleep(0.5)

        if not nav2_ready:
            raise RuntimeError("Nav2 not available - are you running in sim mode without navigation (-#s)? Try full mode (-#f) for exploration tasks.")

        # Wait for Nav2 to fully activate
        self.navigator.waitUntilNav2Active()

        # Create Explorer
        self.explorer = Explorer(self, self.navigator)

        self.get_logger().info("Navigation components ready")

    def _get_current_pose(self) -> PoseStamped:
        """Get the current robot pose."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # In simulation, assume starting at origin
        # In production, use TF2 to get actual pose from /odom or /base_link
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
            # Create temporary directory for map files
            map_dir = f"/tmp/coven_maps/{self.module_id}"
            os.makedirs(map_dir, exist_ok=True)

            map_file = f"{map_dir}/exploration_map"

            # Call map_saver_cli to save the map
            self.get_logger().info(f"Saving map to {map_file}...")

            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", map_file, "--ros-args", "-p", "save_map_timeout:=5000"],
                capture_output=True,
                text=True,
                timeout=10
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
    node = Module()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Graceful shutdown on Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()