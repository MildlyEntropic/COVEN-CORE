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
import json
import threading
import time
import uuid

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import ModuleState


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
        # Stop heartbeat and emit task start
        self.stop_heartbeat()
        self.state = ModuleState.FIELD_OPS
        ts = common.TaskStart(module_id=self.module_id, task=task_name)
        self.pub_task_start.publish(String(data=common.task_start_encode(ts)))
        self.get_logger().info(f"Executing FIELD_OPS for task: {task_name}")

        time.sleep(TASK_DELAY)

        # Return and emit task complete
        self.state = ModuleState.NORMAL
        self.start_heartbeat()
        tc = common.TaskComplete(module_id=self.module_id, task=task_name, success=True, note="Returned from task")
        self.pub_task_complete.publish(String(data=common.task_complete_encode(tc)))
        self.get_logger().info(f"Task {task_name} complete — rejoined dock")

# ------------------------
# --- Main ---
# ------------------------
def main():
    rclpy.init()
    node = Module()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()