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

# --- Third-party ---
import numpy as np

# --- Third-party (ROS2) ---
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

# --- Local (COVEN) ---
import coven_core.common as common
from coven_core.common import (
    COLOR_GREEN, COLOR_YELLOW, COLOR_ORANGE, COLOR_RED, COLOR_RESET,
    BidNotice, BidProposal, CoverageGoal, CoverageStatus, Sector,
    CoverageMissionComplete, MissionRequest
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
DEFAULT_RECHARGE_DELAY = 30.0 # seconds to wait for battery recharge before re-dispatch


# ------------------------
# --- Dock Node ---
# ------------------------
class Dock(Node):
    """Dock node that manages multiple modules and assigns tasks."""

    def __init__(self, dock_name: str = None):
        super().__init__('coven_dock')

        # Declare ROS2 parameters (loaded from config/coven_params.yaml)
        self.declare_parameter('identify_period', DEFAULT_IDENT_PERIOD)
        self.declare_parameter('heartbeat_timeout', DEFAULT_HB_TIMEOUT)
        self.declare_parameter('heartbeat_jitter', DEFAULT_HB_JITTER)
        self.declare_parameter('max_heartbeat_misses', DEFAULT_MAX_MISSES)
        self.declare_parameter('map_storage_dir', DEFAULT_MAP_STORAGE_DIR)
        self.declare_parameter('bid_deadline', DEFAULT_BID_DEADLINE)
        self.declare_parameter('recharge_delay', DEFAULT_RECHARGE_DELAY)
        self.declare_parameter('dock_name', '')

        # Get parameter values
        self.ident_period = self.get_parameter('identify_period').value
        self.hb_timeout = self.get_parameter('heartbeat_timeout').value
        self.hb_jitter = self.get_parameter('heartbeat_jitter').value
        self.max_misses = int(self.get_parameter('max_heartbeat_misses').value)
        self.bid_deadline = self.get_parameter('bid_deadline').value
        self.recharge_delay = self.get_parameter('recharge_delay').value

        # Dock name: use parameter, or constructor arg, or generate coven name
        dock_name_param = self.get_parameter('dock_name').value
        self.dock_name = dock_name_param or dock_name or common.get_coven_name()

        self.modules = {}  # module_id → {state, last_hb, miss_count, paused}
        self.live_hb = set()
        self._mod_lock = threading.Lock()

        # Bidding system state
        self.pending_auctions = {}  # task_id → {task, bids: [], deadline_timer, start_time}
        self._auction_lock = threading.Lock()

        # Map storage directory (from parameter)
        self.map_storage_dir = os.path.expanduser(self.get_parameter('map_storage_dir').value)
        os.makedirs(self.map_storage_dir, exist_ok=True)

        # ROS Topics - Use absolute paths (/coven/...) so they work globally
        # regardless of node namespace. This allows non-namespaced dock to communicate
        # with namespaced module nodes.
        self.pub_ident_req = self.create_publisher(String, '/coven/identify_req', 10)
        self.sub_ident_rep = self.create_subscription(String, '/coven/identify_rep', self.on_ident_rep, 10)

        self.pub_verify_req = self.create_publisher(String, '/coven/verify_req', 10)
        self.sub_verify_rep = self.create_subscription(String, '/coven/verify_rep', self.on_verify_rep, 10)

        self.pub_enable_12v = self.create_publisher(String, '/coven/enable_12v', 10)
        self.sub_hb = self.create_subscription(String, '/coven/heartbeat', self.on_hb, 10)

        self.sub_mission_req = self.create_subscription(String, '/coven/mission_req', self.on_mission_req, 10)
        self.sub_task_ack = self.create_subscription(String, '/coven/task_ack', self.on_task_ack, 10)
        self.sub_task_start = self.create_subscription(String, '/coven/task_start', self.on_task_start, 10)
        self.sub_task_complete = self.create_subscription(String, '/coven/task_complete', self.on_task_complete, 10)

        self.pub_task_req = self.create_publisher(String, '/coven/task_req', 10)

        # Bidding system topics
        self.pub_bid_notice = self.create_publisher(String, '/coven/bid_notice', 10)
        self.sub_bid_proposal = self.create_subscription(String, '/coven/bid_proposal', self.on_bid_proposal, 10)

        # Module ready announcements (immediate IDENTIFY response)
        self.sub_module_ready = self.create_subscription(String, '/coven/module_ready', self.on_module_ready, 10)

        # Coverage exploration state
        self.coverage_mission_active = False
        self.coverage_target = 0.95  # Default target
        self.coverage_goal = None  # Current CoverageGoal
        self.global_coverage = 0.0  # Merged coverage from all rovers
        self.rover_coverage = {}  # module_id → latest coverage percentage
        self.rover_maps = {}  # module_id → latest map data (for merging)
        self.sectors = []  # List of Sector objects
        self.dispatch_cycles = 0  # How many times rovers have been re-dispatched
        self.recharge_timers = {}  # module_id → Timer for pending recharge re-dispatch
        self._coverage_lock = threading.Lock()

        # Coverage status subscription
        self.sub_coverage_status = self.create_subscription(
            String, '/coven/coverage_status', self.on_coverage_status, 10
        )

        # Subscribe to SLAM map for global coverage calculation
        # Rovers return raw LiDAR data, dock runs SLAM, dock reads the map
        self.current_map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10
        )

        self.ident_timer = self.create_timer(self.ident_period, self.broadcast_identify)
        self.hb_timer = self.create_timer(0.5, self.flush_heartbeat_log)

        self.get_logger().info(
            f"Coven '{self.dock_name}' initialized — ready to manage witches "
            f"(ident_period: {self.ident_period}s, bid_deadline: {self.bid_deadline}s)"
        )

    # ------------------------
    # IDENTIFY / VERIFY
    # ------------------------
    def on_module_ready(self, msg: String):
        """Handle module ready announcement - send immediate IDENTIFY_REQ."""
        try:
            data = json.loads(msg.data)
            module_id = data.get("module_id", "unknown")
            self.get_logger().info(f"Module '{module_id}' announced ready — sending IDENTIFY_REQ")
            # Send immediate IDENTIFY to this module
            self.broadcast_identify()
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid module_ready message format")

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
        self.get_logger().info(f"{COLOR_GREEN}Mission request received: {msg.data[:100]}...{COLOR_RESET}")
        req = common.mission_req_decode(msg)
        if not req or not req.task:
            self.get_logger().warn(f"Received invalid mission request. req={req}, task={req.task if req else 'None'}")
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

        # Check if this is a coverage mission
        if req.task == "coverage" and req.coverage_goal:
            self.get_logger().info(
                f"{COLOR_GREEN}Received coverage exploration mission: "
                f"target={req.coverage_goal.target_coverage:.0%}{COLOR_RESET}"
            )
            # Coverage missions use direct dispatch, not auction
            self._start_coverage_mission(req.coverage_goal)
            return

        # For waypoint missions, encode waypoints into task data
        if req.task == "explore" and req.waypoints:
            waypoint_count = len(req.waypoints)
            self.get_logger().info(
                f"{COLOR_GREEN}Received exploration mission with {waypoint_count} waypoints{COLOR_RESET}"
            )
            # Encode full mission as JSON task string for module
            task_data = common.mission_req_encode(req)
        else:
            task_data = req.task

        # Start an auction for this task
        task_id = f"task_{uuid.uuid4().hex[:8]}"
        self._start_auction(task_id, task_data)

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

        # Handle coverage mission completion
        if self.coverage_mission_active:
            self._handle_coverage_task_complete(tc)

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
    # SLAM MAP COVERAGE CALCULATION
    # ------------------------
    def _map_callback(self, msg: OccupancyGrid):
        """Update current map from SLAM and recalculate global coverage."""
        self.current_map = msg
        self._update_global_coverage()

    def _update_global_coverage(self):
        """Calculate global coverage from SLAM occupancy grid.

        Coverage = known cells / total cells, where known means free (0) or
        occupied (100), not unknown (-1).
        """
        if self.current_map is None:
            return

        data = np.array(self.current_map.data)
        # Known cells: value >= 0 (free=0, occupied=100, unknown=-1)
        known = np.sum(data >= 0)
        total = len(data)

        with self._coverage_lock:
            old_coverage = self.global_coverage
            self.global_coverage = known / total if total > 0 else 0.0

            # Log significant coverage changes (>5%)
            if abs(self.global_coverage - old_coverage) > 0.05:
                self.get_logger().info(
                    f"SLAM map coverage: {self.global_coverage:.1%} "
                    f"({known}/{total} cells known)"
                )

    # ------------------------
    # COVERAGE MISSION COORDINATION
    # ------------------------
    def on_coverage_status(self, msg: String):
        """Handle periodic coverage status updates from rovers."""
        status = common.coverage_status_decode(msg)
        if not status:
            return

        with self._coverage_lock:
            # Update rover's latest coverage
            self.rover_coverage[status.module_id] = status.current_coverage

            self.get_logger().info(
                f"Coverage status from {status.module_id}: "
                f"{status.current_coverage:.1%} coverage, "
                f"{status.battery_remaining:.0%} battery, "
                f"{status.frontiers_remaining} frontiers"
                + (f" (returning: {status.reason})" if status.returning_to_dock else "")
            )

    def _start_coverage_mission(self, goal: CoverageGoal):
        """
        Start a multi-rover coverage exploration mission.

        Divides the arena into sectors and dispatches rovers to explore.
        """
        self.get_logger().info(
            f"{COLOR_GREEN}Starting coverage mission: "
            f"target={goal.target_coverage:.0%}, max_time={goal.max_exploration_time}s{COLOR_RESET}"
        )

        with self._coverage_lock:
            self.coverage_mission_active = True
            self.coverage_target = goal.target_coverage
            self.coverage_goal = goal
            self.global_coverage = 0.0
            self.rover_coverage = {}
            self.rover_maps = {}
            self.dispatch_cycles = 0

        # Get available rovers and assign sectors
        available_rovers = self._get_available_rovers()
        if not available_rovers:
            self.get_logger().warn("No available rovers for coverage mission")
            return

        # Divide arena into sectors based on number of rovers
        self.sectors = self._divide_into_sectors(len(available_rovers), goal.sector_bounds)

        # Dispatch each rover to its sector
        for rover_id, sector in zip(available_rovers, self.sectors):
            sector.assigned_to = rover_id
            self._dispatch_coverage_task(rover_id, sector, goal)

        self.dispatch_cycles = 1

    def _get_available_rovers(self) -> list:
        """Get list of module IDs that are available for tasks."""
        available = []
        with self._mod_lock:
            for mod_id, mod in self.modules.items():
                if mod["state"] == common.DockState.ENABLED and not mod["paused"]:
                    available.append(mod_id)
        return available

    def _divide_into_sectors(self, num_rovers: int, bounds: tuple = None) -> list:
        """
        Divide the exploration area into sectors for rovers.

        Uses simple quadrant division for 2-4 rovers.
        For more rovers, uses grid-based division.

        Args:
            num_rovers: Number of rovers to create sectors for
            bounds: Optional (x_min, y_min, x_max, y_max), defaults to 15m x 15m arena

        Returns:
            List of Sector objects
        """
        # Default to 15m x 15m arena centered at origin
        if bounds:
            x_min, y_min, x_max, y_max = bounds
        else:
            x_min, y_min, x_max, y_max = -7.5, -7.5, 7.5, 7.5

        width = x_max - x_min
        height = y_max - y_min
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2

        sectors = []

        if num_rovers == 1:
            # Single rover - entire area
            sectors.append(Sector(
                name="ALL",
                bounds=(x_min, y_min, x_max, y_max)
            ))
        elif num_rovers == 2:
            # Two rovers - split horizontally (east/west)
            sectors.append(Sector(
                name="WEST",
                bounds=(x_min, y_min, cx, y_max)
            ))
            sectors.append(Sector(
                name="EAST",
                bounds=(cx, y_min, x_max, y_max)
            ))
        elif num_rovers <= 4:
            # 3-4 rovers - quadrants
            quadrants = [
                ("NE", (cx, cy, x_max, y_max)),
                ("NW", (x_min, cy, cx, y_max)),
                ("SW", (x_min, y_min, cx, cy)),
                ("SE", (cx, y_min, x_max, cy)),
            ]
            for i, (name, b) in enumerate(quadrants[:num_rovers]):
                sectors.append(Sector(name=name, bounds=b))
        else:
            # More rovers - grid division
            cols = int(num_rovers ** 0.5)
            rows = (num_rovers + cols - 1) // cols
            cell_w = width / cols
            cell_h = height / rows

            idx = 0
            for row in range(rows):
                for col in range(cols):
                    if idx >= num_rovers:
                        break
                    sx = x_min + col * cell_w
                    sy = y_min + row * cell_h
                    sectors.append(Sector(
                        name=f"G{row}_{col}",
                        bounds=(sx, sy, sx + cell_w, sy + cell_h)
                    ))
                    idx += 1

        self.get_logger().info(
            f"Divided arena into {len(sectors)} sectors: "
            f"{[s.name for s in sectors]}"
        )
        return sectors

    def _dispatch_coverage_task(self, rover_id: str, sector: Sector, goal: CoverageGoal):
        """
        Dispatch a coverage exploration task to a specific rover.

        Args:
            rover_id: Module ID of the rover
            sector: Sector assigned to this rover
            goal: Original coverage goal
        """
        # Create sector-specific goal
        sector_goal = CoverageGoal(
            target_coverage=goal.target_coverage,
            sector=sector.name,
            sector_bounds=sector.bounds,
            max_exploration_time=goal.max_exploration_time,
            return_on_low_battery=goal.return_on_low_battery,
            battery_return_threshold=goal.battery_return_threshold,
        )

        # Create mission request with coverage goal
        mission = MissionRequest(
            task="coverage",
            coverage_goal=sector_goal,
            return_to_dock=True
        )

        # Encode as JSON task string
        task_data = common.mission_req_encode(mission)

        self.get_logger().info(
            f"{COLOR_YELLOW}Dispatching {rover_id} to sector {sector.name} "
            f"(bounds: {sector.bounds}){COLOR_RESET}"
        )

        # Mark rover as busy
        with self._mod_lock:
            mod = self.modules.get(rover_id)
            if mod:
                mod["paused"] = True

        # Send task request directly (no auction for assigned sectors)
        task_req = common.TaskReq(module_id=rover_id, task=task_data)
        self.pub_task_req.publish(String(data=common.task_req_encode(task_req)))

    def _handle_coverage_task_complete(self, tc: common.TaskComplete):
        """
        Handle task completion during a coverage mission.

        Merges map data and checks if global coverage target is met.
        Re-dispatches rover if more exploration is needed.
        """
        with self._coverage_lock:
            if not self.coverage_mission_active:
                return

            # Store rover's reported coverage (for logging only - Rust rovers report 0.0)
            rover_coverage = tc.exploration_metrics.get("coverage", 0.0)
            self.rover_coverage[tc.module_id] = rover_coverage

            # Store map data if present (legacy - not used with SLAM-based coverage)
            if tc.map_data:
                self.rover_maps[tc.module_id] = tc.map_data

            # Global coverage is calculated from SLAM map via _map_callback
            # No averaging of rover-reported values (Rust rovers report 0.0)
            self.get_logger().info(
                f"{COLOR_GREEN}Rover {tc.module_id} returned. "
                f"Global coverage from SLAM: {self.global_coverage:.1%} "
                f"(target: {self.coverage_target:.1%}){COLOR_RESET}"
            )

            # Check if target coverage is met
            if self.global_coverage >= self.coverage_target:
                self._complete_coverage_mission(success=True)
                return

            # Check if rover should be re-dispatched
            return_reason = tc.exploration_metrics.get("return_reason", "")

            if return_reason == "low_battery":
                # Rover returned due to low battery — wait for recharge then re-dispatch
                rover_id = tc.module_id
                self.get_logger().info(
                    f"{rover_id} returned for battery recharge - "
                    f"re-dispatch in {self.recharge_delay:.0f}s"
                )
                # Cancel any existing recharge timer for this rover
                if rover_id in self.recharge_timers:
                    self.recharge_timers[rover_id].cancel()
                # One-shot timer: re-dispatch after recharge_delay seconds
                self.recharge_timers[rover_id] = self.create_timer(
                    self.recharge_delay,
                    lambda rid=rover_id: self._recharge_complete(rid),
                )

            elif return_reason == "no_frontiers":
                # This rover's sector is fully explored
                self.get_logger().info(
                    f"{tc.module_id} completed sector - no more frontiers"
                )
                # Find unexplored sectors from other rovers
                self._reassign_to_unexplored_sector(tc.module_id)

            else:
                # Re-dispatch to continue exploration
                self._redispatch_rover(tc.module_id)

    def _redispatch_rover(self, rover_id: str):
        """Re-dispatch a rover to continue coverage exploration."""
        with self._coverage_lock:
            if not self.coverage_mission_active or not self.coverage_goal:
                return

            # Find this rover's assigned sector
            sector = None
            for s in self.sectors:
                if s.assigned_to == rover_id:
                    sector = s
                    break

            if not sector:
                self.get_logger().warn(f"No sector assigned to {rover_id}")
                return

            self.dispatch_cycles += 1

            self.get_logger().info(
                f"{COLOR_YELLOW}Re-dispatching {rover_id} to sector {sector.name} "
                f"(dispatch cycle {self.dispatch_cycles}){COLOR_RESET}"
            )

            self._dispatch_coverage_task(rover_id, sector, self.coverage_goal)

    def _recharge_complete(self, rover_id: str):
        """Called after recharge delay expires — cancel the timer and re-dispatch."""
        if rover_id in self.recharge_timers:
            self.recharge_timers[rover_id].cancel()
            del self.recharge_timers[rover_id]
        self.get_logger().info(
            f"{COLOR_GREEN}{rover_id} recharge complete - re-dispatching{COLOR_RESET}"
        )
        self._redispatch_rover(rover_id)

    def _split_sector(self, sector: Sector, rover_id: str) -> Sector:
        """Split a sector along its longer axis, returning the new half for rover_id."""
        x_min, y_min, x_max, y_max = sector.bounds
        width = x_max - x_min
        height = y_max - y_min

        if width >= height:
            x_mid = x_min + width / 2
            sector.bounds = (x_min, y_min, x_mid, y_max)
            new_bounds = (x_mid, y_min, x_max, y_max)
            split_axis = "X"
        else:
            y_mid = y_min + height / 2
            sector.bounds = (x_min, y_min, x_max, y_mid)
            new_bounds = (x_min, y_mid, x_max, y_max)
            split_axis = "Y"

        new_sector = Sector(
            name=f"{sector.name}_{rover_id[:8]}",
            bounds=new_bounds,
            assigned_to=rover_id,
        )
        self.sectors.append(new_sector)

        self.get_logger().info(
            f"Split sector {sector.name} along {split_axis}-axis: "
            f"{sector.assigned_to} keeps {sector.bounds}, "
            f"{rover_id} gets {new_bounds}"
        )
        return new_sector

    def _reassign_to_unexplored_sector(self, rover_id: str):
        """
        Reassign a rover to help with an unexplored sector.

        Called when rover's original sector is fully explored.
        """
        with self._coverage_lock:
            if not self.coverage_mission_active:
                return

            # Find sectors with lowest coverage
            least_covered = None
            min_coverage = 1.0

            for sector in self.sectors:
                if sector.assigned_to and sector.assigned_to != rover_id:
                    other_coverage = self.rover_coverage.get(sector.assigned_to, 0.0)
                    if other_coverage < min_coverage:
                        min_coverage = other_coverage
                        least_covered = sector

            if least_covered and min_coverage < self.coverage_target:
                new_sector = self._split_sector(least_covered, rover_id)
                if self.coverage_goal:
                    self._dispatch_coverage_task(rover_id, new_sector, self.coverage_goal)
            else:
                self.get_logger().info(
                    f"{rover_id} completed exploration - all sectors covered"
                )

    def _complete_coverage_mission(self, success: bool = True):
        """Complete the coverage mission and log final statistics."""
        with self._coverage_lock:
            self.coverage_mission_active = False

            mission_complete = CoverageMissionComplete(
                success=success,
                total_coverage=self.global_coverage,
                target_coverage=self.coverage_target,
                rovers_dispatched=len(self.rover_coverage),
                dispatch_cycles=self.dispatch_cycles,
            )

            result_str = "SUCCESS" if success else "INCOMPLETE"
            self.get_logger().info(
                f"{COLOR_GREEN}{'='*50}\n"
                f"COVERAGE MISSION {result_str}\n"
                f"  Target: {self.coverage_target:.0%}\n"
                f"  Achieved: {self.global_coverage:.1%}\n"
                f"  Rovers: {len(self.rover_coverage)}\n"
                f"  Dispatch cycles: {self.dispatch_cycles}\n"
                f"{'='*50}{COLOR_RESET}"
            )


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