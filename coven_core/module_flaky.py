"""
module_flaky.py — COVEN Phase 1

ROS2 node simulating a flaky COVEN-compliant module.
Behaves like a normal module, but periodically skips
1–2 heartbeats, then recovers — never enough to be dropped.

Used to test DockMulti’s heartbeat recovery logic:
YELLOW → ORANGE → GREEN without triggering disconnect.

Note: This module logs only initialization and task replies.
All heartbeat recovery and dropout logs appear on the Dock side.

Author: Alexander Shultis
Date: September 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
# --- Standard library ---
import random

# --- Third-party (ROS2) ---
import rclpy
from std_msgs.msg import String

# --- Local (COVEN) ---
from coven_core.module_node import Module
import coven_core.common as common


# ------------------------
# --- Flaky Module Node ---
# ------------------------
class FlakyModule(Module):
    """Module with intermittent heartbeat skips, simulating flaky behavior."""

    def __init__(self, module_id=None, module_type="FlakyRover", fw="0.0.1-flaky"):
        super().__init__(module_id=module_id, module_type=module_type, fw=fw)

        self.skip_target = 0     # total skips for this cycle
        self.skip_count = 0      # how many skips so far
        self.cycle_count = 0     # how many flaky cycles done

        self.get_logger().info(f"FlakyModule {self.module_id} initialized.")

    def send_hb(self):
        """Override heartbeat to occasionally skip 1–2 beats (never 3)."""
        if self.state != common.ModuleState.NORMAL:
            return

        if self.skip_count < self.skip_target:
            self.skip_count += 1
            return

        # Normal heartbeat
        self.seq += 1
        hb = common.Heartbeat(module_id=self.module_id, seq=self.seq)
        self.pub_hb.publish(String(data=common.hb_encode(hb)))

        # Randomly start a skip cycle
        if random.random() < 0.2 and self.cycle_count < 5:
            self.skip_target = random.choice([1, 2])
            self.skip_count = 0
            self.cycle_count += 1


# ------------------------
# --- Main ---
# ------------------------
def main():
    """Entrypoint for FlakyModule."""
    rclpy.init()
    node = FlakyModule()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()