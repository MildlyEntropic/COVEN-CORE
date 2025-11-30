"""
dock_node.py — COVEN Phase 1

ROS2 node representing a docking hub that manages multiple modules.
Handles multiple modules concurrently with independent FSM states.

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
import base64
import gzip
import json
import os
import threading
import uuid
from datetime import datetime

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import (
    COLOR_GREEN, COLOR_YELLOW, COLOR_ORANGE, COLOR_RED, COLOR_RESET,
    BidNotice, BidProposal
)

# ------------------------
# --- Constants ---
# ------------------------
# NOTE: These are now ROS2 parameters - defaults defined here for reference
# Actual values loaded from config/coven_params.yaml
DEFAULT_IDENT_PERIOD = 5.0    # seconds between IDENTIFY broadcasts
DEFAULT_HB_TIMEOUT   = 1.0    # expected heartbeat interval
DEFAULT_MAX_MISSES   = 3      # after 3 misses, mark module as dropped
DEFAULT_HB_JITTER    = 0.15   # jitter margin
DEFAULT_MAP_STORAGE_DIR = '~/coven_maps'  # map storage location
DEFAULT_BID_DEADLINE = 2.0    # seconds to wait for bids


# ------------------------
# --- Dock Node ---
# ------------------------
class Dock(Node):
    """Dock node that manages multiple modules and assigns tasks."""

    def __init__(self):
        super().__init__('coven_dock')

        # Declare ROS2 parameters (loaded from config/coven_params.yaml)
        self.declare_parameter('identify_period', DEFAULT_IDENT_PERIOD)
        self.declare_parameter('heartbeat_timeout', DEFAULT_HB_TIMEOUT)
        self.declare_parameter('heartbeat_jitter', DEFAULT_HB_JITTER)
        self.declare_parameter('max_heartbeat_misses', DEFAULT_MAX_MISSES)
        self.declare_parameter('map_storage_dir', DEFAULT_MAP_STORAGE_DIR)
        self.declare_parameter('bid_deadline', DEFAULT_BID_DEADLINE)

        # Get parameter values
        self.ident_period = self.get_parameter('identify_period').value
        self.hb_timeout = self.get_parameter('heartbeat_timeout').value
        self.hb_jitter = self.get_parameter('heartbeat_jitter').value
        self.max_misses = int(self.get_parameter('max_heartbeat_misses').value)
        self.bid_deadline = self.get_parameter('bid_deadline').value

        self.modules = {}  # module_id → {state, last_hb, miss_count, paused}
        self.live_hb = set()
        self._mod_lock = threading.Lock()

        # Bidding system state
        self.pending_auctions = {}  # task_id → {task, bids: [], deadline_timer, start_time}
        self._auction_lock = threading.Lock()

        # Map storage directory (from parameter)
        self.map_storage_dir = os.path.expanduser(self.get_parameter('map_storage_dir').value)
        os.makedirs(self.map_storage_dir, exist_ok=True)

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

        # Bidding system topics
        self.pub_bid_notice = self.create_publisher(String, 'coven/bid_notice', 10)
        self.sub_bid_proposal = self.create_subscription(String, 'coven/bid_proposal', self.on_bid_proposal, 10)

        self.ident_timer = self.create_timer(self.ident_period, self.broadcast_identify)
        self.hb_timer = self.create_timer(0.5, self.flush_heartbeat_log)

        self.get_logger().info(f"DockMulti initialized — ready to manage multiple modules (ident_period: {self.ident_period}s, bid_deadline: {self.bid_deadline}s)")

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
                "last_hb": self.get_clock().now().nanoseconds / 1e9,
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
            mod["last_hb"] = self.get_clock().now().nanoseconds / 1e9
            mod["miss_count"] = 0
            self.live_hb.add(hb.module_id)
        if was_missing:
            self.get_logger().info(f"{COLOR_GREEN}Heartbeat recovered for {hb.module_id}{COLOR_RESET}")

    def flush_heartbeat_log(self):
        now = self.get_clock().now().nanoseconds / 1e9
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
        missed = int((dt + self.hb_jitter) // self.hb_timeout)
        if missed > mod["miss_count"]:
            mod["miss_count"] = missed
            if missed == 1:
                self.get_logger().warn(f"{COLOR_YELLOW}Heartbeat missing ONCE for {module_id}{COLOR_RESET}")
            elif missed == 2:
                self.get_logger().warn(f"{COLOR_ORANGE}Heartbeat missing TWICE for {module_id}{COLOR_RESET}")
            elif missed >= self.max_misses:
                self.get_logger().error(f"{COLOR_RED}Heartbeat lost from {module_id}{COLOR_RESET}")
                to_remove.append(module_id)

    # ------------------------
    # BIDDING SYSTEM
    # ------------------------
    def on_mission_req(self, msg: String):
        """Handle incoming mission request by starting an auction."""
        req = common.mission_req_decode(msg)
        if not req or not req.task:
            self.get_logger().warn("Received invalid mission request.")
            return

        # Check if any modules are available
        available_count = 0
        with self._mod_lock:
            for mod in self.modules.values():
                if mod["state"] == common.DockState.ENABLED and not mod["paused"]:
                    available_count += 1

        if available_count == 0:
            self.get_logger().warn("No available modules to bid on mission.")
            return

        # Start an auction for this task
        task_id = f"task_{uuid.uuid4().hex[:8]}"
        self._start_auction(task_id, req.task)

    def _start_auction(self, task_id: str, task: str):
        """Broadcast a BidNotice and start deadline timer."""
        self.get_logger().info(
            f"{COLOR_YELLOW}Starting auction for '{task}' (task_id: {task_id}, deadline: {self.bid_deadline}s){COLOR_RESET}"
        )

        # Create auction record
        with self._auction_lock:
            self.pending_auctions[task_id] = {
                "task": task,
                "bids": [],
                "start_time": self.get_clock().now().nanoseconds / 1e9
            }

        # Broadcast BidNotice
        notice = BidNotice(task_id=task_id, task=task, deadline=self.bid_deadline)
        self.pub_bid_notice.publish(String(data=common.bid_notice_encode(notice)))

        # Start deadline timer
        self.create_timer(
            self.bid_deadline,
            lambda: self._resolve_auction(task_id),
            callback_group=None
        )

    def on_bid_proposal(self, msg: String):
        """Handle incoming bid proposals from modules."""
        proposal = common.bid_proposal_decode(msg)
        if not proposal:
            return

        with self._auction_lock:
            auction = self.pending_auctions.get(proposal.task_id)
            if not auction:
                self.get_logger().debug(f"Received bid for unknown/closed auction: {proposal.task_id}")
                return

            # Record the bid
            auction["bids"].append(proposal)

            if proposal.can_execute:
                self.get_logger().info(
                    f"Bid received: {proposal.module_id} offers cost={proposal.cost:.2f} for {proposal.task_id}"
                )
            else:
                self.get_logger().info(
                    f"Bid declined: {proposal.module_id} cannot execute {proposal.task_id}: {proposal.reason}"
                )

    def _resolve_auction(self, task_id: str):
        """Deadline reached - pick winner and assign task."""
        with self._auction_lock:
            auction = self.pending_auctions.pop(task_id, None)

        if not auction:
            return  # Already resolved or cancelled

        task = auction["task"]
        bids = auction["bids"]

        # Filter to executable bids
        valid_bids = [b for b in bids if b.can_execute]

        if not valid_bids:
            self.get_logger().warn(
                f"{COLOR_ORANGE}Auction failed for '{task}' - no valid bids received "
                f"({len(bids)} total bids, 0 executable){COLOR_RESET}"
            )
            return

        # Select lowest cost bid
        winner = min(valid_bids, key=lambda b: b.cost)

        self.get_logger().info(
            f"{COLOR_GREEN}Auction resolved: '{task}' awarded to {winner.module_id} "
            f"(cost={winner.cost:.2f}, {len(valid_bids)} valid bids){COLOR_RESET}"
        )

        # Mark module as paused (busy)
        with self._mod_lock:
            if winner.module_id in self.modules:
                self.modules[winner.module_id]["paused"] = True

        # Send TaskReq to winner
        task_req = common.TaskReq(module_id=winner.module_id, task=task)
        self.pub_task_req.publish(String(data=common.task_req_encode(task_req)))

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
                mod["last_hb"] = self.get_clock().now().nanoseconds / 1e9
                mod["miss_count"] = 0
        result = "SUCCESS" if tc.success else "FAIL"

        # Log metrics
        metrics_str = ""
        if tc.exploration_metrics:
            metrics_str = (
                f" | Coverage: {tc.exploration_metrics.get('coverage', 0):.1%}, "
                f"Duration: {tc.exploration_metrics.get('duration', 0):.1f}s, "
                f"Iterations: {tc.exploration_metrics.get('iterations', 0)}"
            )

        self.get_logger().info(
            f"{COLOR_GREEN}TaskComplete from {tc.module_id}: {tc.task} → {result}{metrics_str}{COLOR_RESET}"
        )

        # Save map data if present
        if tc.map_data or tc.map_yaml:
            self._save_map_data(tc)

    def _save_map_data(self, tc: common.TaskComplete):
        """
        Deserialize and save map data received from module.

        Args:
            tc: TaskComplete message containing map data
        """
        try:
            # Create timestamped directory for this mission
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            mission_dir = os.path.join(
                self.map_storage_dir,
                tc.module_id,
                f"{tc.task}_{timestamp}"
            )
            os.makedirs(mission_dir, exist_ok=True)

            # Decode and decompress map data
            if tc.map_data:
                pgm_compressed = base64.b64decode(tc.map_data)
                pgm_data = gzip.decompress(pgm_compressed)

                pgm_file = os.path.join(mission_dir, "exploration_map.pgm")
                with open(pgm_file, 'wb') as f:
                    f.write(pgm_data)

                self.get_logger().info(f"Saved map PGM: {pgm_file} ({len(pgm_data)} bytes)")

            # Decode and decompress YAML metadata
            if tc.map_yaml:
                yaml_compressed = base64.b64decode(tc.map_yaml)
                yaml_data = gzip.decompress(yaml_compressed)

                yaml_file = os.path.join(mission_dir, "exploration_map.yaml")
                with open(yaml_file, 'wb') as f:
                    f.write(yaml_data)

                self.get_logger().info(f"Saved map YAML: {yaml_file} ({len(yaml_data)} bytes)")

            # Save exploration metrics as JSON
            if tc.exploration_metrics:
                metrics_file = os.path.join(mission_dir, "metrics.json")
                with open(metrics_file, 'w') as f:
                    json.dump({
                        "module_id": tc.module_id,
                        "task": tc.task,
                        "timestamp": timestamp,
                        "success": tc.success,
                        "note": tc.note,
                        "metrics": tc.exploration_metrics
                    }, f, indent=2)

                self.get_logger().info(f"Saved metrics: {metrics_file}")

            self.get_logger().info(
                f"{COLOR_GREEN}Map data from {tc.module_id} saved to {mission_dir}{COLOR_RESET}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to save map data: {e}")


# ------------------------
# --- Main ---
# ------------------------
def main():
    rclpy.init()
    node = Dock()
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