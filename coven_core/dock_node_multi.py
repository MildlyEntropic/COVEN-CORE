"""
dock_node_multi.py — COVEN Phase 1

ROS2 node representing a multi-slot docking hub.
Extends the single Dock FSM to handle multiple modules concurrently.

Responsibilities:
- Broadcast IDENTIFY_REQ and track module responses.
- Verify modules and power-enable them.
- Monitor periodic heartbeat from each module.
- Receive high-level mission request.
- Assign a single ENABLED module to carry it out.
- Track task lifecycle: TaskReq → TaskAck → TaskStart → TaskComplete.

Author: Alexander Shultis
Date: September 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
# --- Standard library ---
import json
import time
import threading

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import (
    COLOR_GREEN, COLOR_YELLOW, COLOR_ORANGE, COLOR_RED, COLOR_RESET
)

# ------------------------
# --- Constants ---
# ------------------------
IDENT_PERIOD = 5.0    # seconds between IDENTIFY broadcasts
HB_PERIOD    = 1.0    # expected heartbeat interval
MAX_MISSES   = 3      # after 3 misses, mark module as dropped
EPS          = 0.15   # jitter margin


# ------------------------
# --- DockMulti Node ---
# ------------------------
class DockMulti(Node):
    """Dock node that manages multiple modules and assigns tasks."""

    def __init__(self):
        super().__init__('coven_dock_multi')

        self.modules = {}  # module_id → {state, last_hb, miss_count, paused}
        self.live_hb = set()
        self._mod_lock = threading.Lock()

        # ROS Topics
        self.pub_ident_req = self.create_publisher(String, 'coven/identify_req', 10)
        self.sub_ident_rep = self.create_subscription(String, 'coven/identify_rep', self.on_ident_rep, 10)

        self.pub_verify_req = self.create_publisher(String, 'coven/verify_req', 10)
        self.sub_verify_rep = self.create_subscription(String, 'coven/verify_rep', self.on_verify_rep, 10)

        self.pub_enable_12v = self.create_publisher(String, 'coven/enable_12v', 10)
        self.sub_hb = self.create_subscription(String, 'coven/heartbeat', self.on_hb, 10)

        self.sub_mission_req = self.create_subscription(String, 'coven/mission_req', self.on_mission_req, 10)
        self.sub_task_ack = self.create_subscription(String, 'coven/task_ack', self.on_task_ack, 10)
        self.sub_task_start = self.create_subscription(String, 'coven/task_start', self.on_task_start, 10)
        self.sub_task_complete = self.create_subscription(String, 'coven/task_complete', self.on_task_complete, 10)

        self.pub_task_req = self.create_publisher(String, 'coven/task_req', 10)

        self.ident_timer = self.create_timer(IDENT_PERIOD, self.broadcast_identify)
        self.hb_timer = self.create_timer(0.5, self.flush_heartbeat_log)

        self.get_logger().info("DockMulti initialized — ready to manage multiple modules.")

    # ------------------------
    # IDENTIFY / VERIFY
    # ------------------------
    def broadcast_identify(self):
        req = common.IdentifyReq(req_id="dock_broadcast")
        self.pub_ident_req.publish(String(data=common.ident_req_encode(req)))
        self.get_logger().info("Broadcast IDENTIFY_REQ dock_broadcast")

    def on_ident_rep(self, msg: String):
        rep = common.ident_rep_decode(msg)
        if not rep:
            return
        self.get_logger().info(f"IDENTIFY_REP received from {rep.module_id}")
        with self._mod_lock:
            self.modules[rep.module_id] = {
                "state": common.DockState.VERIFY,
                "last_hb": time.time(),
                "miss_count": 0,
                "paused": False
            }
        verify = common.VerifyReq(module_id=rep.module_id)
        self.pub_verify_req.publish(String(data=common.verify_req_encode(verify)))

    def on_verify_rep(self, msg: String):
        rep = common.verify_rep_decode(msg)
        if not rep:
            return
        with self._mod_lock:
            if rep.module_id not in self.modules:
                return
            if rep.ok:
                self.modules[rep.module_id]["state"] = common.DockState.ENABLED
            else:
                self.modules[rep.module_id]["state"] = common.DockState.REJECTED
        if rep.ok:
            self.get_logger().info(f"VERIFY_REP OK for {rep.module_id} → enabling +12V")
            self.pub_enable_12v.publish(
                String(data=json.dumps({"module_id": rep.module_id, "data": True}))
            )
        else:
            self.get_logger().warn(f"VERIFY_REP failed for {rep.module_id}: {rep.reason}")

    # ------------------------
    # HEARTBEAT MONITORING
    # ------------------------
    def on_hb(self, msg: String):
        hb = common.hb_decode(msg)
        if not hb:
            return
        with self._mod_lock:
            mod = self.modules.get(hb.module_id)
            if not mod:
                return
            was_missing = mod["miss_count"] > 0
            mod["last_hb"] = time.time()
            mod["miss_count"] = 0
            self.live_hb.add(hb.module_id)
        if was_missing:
            self.get_logger().info(f"{COLOR_GREEN}Heartbeat recovered for {hb.module_id}{COLOR_RESET}")

    def flush_heartbeat_log(self):
        now = time.time()
        with self._mod_lock:
            live_snapshot = sorted(self.live_hb)
            self.live_hb.clear()
        if live_snapshot:
            self.get_logger().info(f"{COLOR_GREEN}Heartbeat received for {', '.join(live_snapshot)}{COLOR_RESET}")
        to_remove = []
        with self._mod_lock:
            for module_id, mod in self.modules.items():
                if mod.get("paused"):
                    continue
                self._check_heartbeat(now, module_id, mod, to_remove)
        for mid in to_remove:
            self.modules.pop(mid, None)

    def _check_heartbeat(self, now, module_id, mod, to_remove):
        dt = now - mod["last_hb"]
        missed = int((dt + EPS) // HB_PERIOD)
        if missed > mod["miss_count"]:
            mod["miss_count"] = missed
            if missed == 1:
                self.get_logger().warn(f"{COLOR_YELLOW}Heartbeat missing ONCE for {module_id}{COLOR_RESET}")
            elif missed == 2:
                self.get_logger().warn(f"{COLOR_ORANGE}Heartbeat missing TWICE for {module_id}{COLOR_RESET}")
            elif missed >= MAX_MISSES:
                self.get_logger().error(f"{COLOR_RED}Heartbeat lost from {module_id}{COLOR_RESET}")
                to_remove.append(module_id)

    # ------------------------
    # MISSION TASK FLOW
    # ------------------------
    def on_mission_req(self, msg: String):
        req = common.mission_req_decode(msg)
        if not req or not req.task:
            self.get_logger().warn("Received invalid mission request.")
            return
        chosen = None
        with self._mod_lock:
            for module_id, mod in self.modules.items():
                if mod["state"] == common.DockState.ENABLED and not mod["paused"]:
                    chosen = module_id
                    mod["paused"] = True
                    break
        if chosen:
            task = common.TaskReq(module_id=chosen, task=req.task)
            self.pub_task_req.publish(String(data=common.task_req_encode(task)))
            self.get_logger().info(f"Assigned mission '{req.task}' to {chosen}")
        else:
            self.get_logger().warn("No available module to assign mission.")

    def on_task_ack(self, msg: String):
        ack = common.task_ack_decode(msg)
        if not ack:
            return
        result = "ACCEPTED" if ack.accepted else f"REJECTED ({ack.reason})"
        self.get_logger().info(f"TASK_ACK from {ack.module_id}: {result}")

    def on_task_start(self, msg: String):
        ts = common.task_start_decode(msg)
        if not ts:
            return
        with self._mod_lock:
            mod = self.modules.get(ts.module_id)
            if mod:
                mod["state"] = common.ModuleState.FIELD_OPS
        self.get_logger().info(f"{COLOR_ORANGE}{ts.module_id} started FIELD_OPS: {ts.task}{COLOR_RESET}")

    def on_task_complete(self, msg: String):
        tc = common.task_complete_decode(msg)
        if not tc:
            return
        with self._mod_lock:
            mod = self.modules.get(tc.module_id)
            if mod:
                mod["paused"] = False
                mod["state"] = common.DockState.ENABLED
                mod["last_hb"] = time.time()
                mod["miss_count"] = 0
        result = "SUCCESS" if tc.success else "FAIL"
        self.get_logger().info(f"{COLOR_GREEN}TaskComplete from {tc.module_id}: {tc.task} → {result}{COLOR_RESET}")


# ------------------------
# --- Main ---
# ------------------------
def main():
    rclpy.init()
    node = DockMulti()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()