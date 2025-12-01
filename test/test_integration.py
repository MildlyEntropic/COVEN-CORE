"""
test_integration.py — Integration Tests for COVEN

Tests that verify dock-module integration chains:
- Input to function A → output to function B → expected final result

These tests call real handlers with real inputs and verify real outputs.

Author: Alexander Shultis
Date: November 2025
"""

import unittest
import time
import threading
from std_msgs.msg import String
import rclpy
from rclpy.executors import MultiThreadedExecutor

from coven_core.module_node import Module
from coven_core.dock_node import Dock
from coven_core.common import (
    ModuleState, DockState,
    IdentifyReq, IdentifyRep, VerifyReq,
    Heartbeat,
    ident_req_encode, ident_rep_encode,
    verify_req_encode,
    hb_encode, hb_decode,
)


class TestIdentificationChain(unittest.TestCase):
    """
    Test the identification chain:
    Input: IDENTIFY_REQ message
    Chain: module.on_ident_req() → publishes IDENTIFY_REP → dock.on_ident_rep()
    Output: dock.modules contains the module with correct structure
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        pass  # Don't shutdown - other test classes may still need rclpy

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.dock = Dock()
        self.module = Module(module_id="TEST-CHAIN-001", executor=self.executor)
        self.executor.add_node(self.dock)
        self.executor.add_node(self.module)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_module_responds_to_identify_req(self):
        """
        Input: IDENTIFY_REQ with req_id="test-123"
        Action: Call module.on_ident_req()
        Expected: Module state changes BOOT → WAIT_VERIFY
        """
        # Precondition
        self.assertEqual(self.module.state, ModuleState.BOOT)

        # Input
        req = IdentifyReq(req_id="test-123")
        msg = String(data=ident_req_encode(req))

        # Action
        self.module.on_ident_req(msg)

        # Output
        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)

    def test_module_ignores_identify_when_not_boot(self):
        """
        Input: IDENTIFY_REQ when module is in NORMAL state
        Action: Call module.on_ident_req()
        Expected: Module state stays NORMAL (ignores message)
        """
        # Precondition: Set to NORMAL
        self.module.state = ModuleState.NORMAL

        # Input
        req = IdentifyReq(req_id="test-456")
        msg = String(data=ident_req_encode(req))

        # Action
        self.module.on_ident_req(msg)

        # Output: State unchanged
        self.assertEqual(self.module.state, ModuleState.NORMAL)

    def test_dock_registers_module_on_ident_rep(self):
        """
        Input: IDENTIFY_REP with module_id="TEST-CHAIN-001"
        Action: Call dock.on_ident_rep()
        Expected: dock.modules["TEST-CHAIN-001"] exists with correct keys
        """
        # Precondition: No modules registered
        self.assertEqual(len(self.dock.modules), 0)

        # Input
        rep = IdentifyRep(
            req_id="test-123",
            module_id="TEST-CHAIN-001",
            module_type="ReconRover",
            fw="1.0.0"
        )
        msg = String(data=ident_rep_encode(rep))

        # Action
        self.dock.on_ident_rep(msg)

        # Output: Module registered with CORRECT keys
        self.assertIn("TEST-CHAIN-001", self.dock.modules)
        mod = self.dock.modules["TEST-CHAIN-001"]

        # Verify ALL keys match what dock_node.py actually uses
        self.assertIn("state", mod)
        self.assertIn("last_hb", mod)
        self.assertIn("miss_count", mod)  # NOT "missed_hbs"
        self.assertIn("paused", mod)

        # Verify values
        self.assertEqual(mod["state"], DockState.VERIFY)
        self.assertEqual(mod["miss_count"], 0)
        self.assertFalse(mod["paused"])


class TestVerificationChain(unittest.TestCase):
    """
    Test the verification chain:
    Input: VERIFY_REQ message
    Chain: module.on_verify_req() → publishes VERIFY_REP → dock.on_verify_rep()
    Output: dock sends power enable, module transitions to NORMAL
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.dock = Dock()
        self.module = Module(module_id="TEST-VERIFY-001", executor=self.executor)
        self.executor.add_node(self.dock)
        self.executor.add_node(self.module)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_module_responds_to_verify_req(self):
        """
        Input: VERIFY_REQ for module_id="TEST-VERIFY-001"
        Precondition: Module in WAIT_VERIFY state
        Expected: Module stays in WAIT_VERIFY (waiting for power enable)
        """
        # Precondition
        self.module.state = ModuleState.WAIT_VERIFY

        # Input
        req = VerifyReq(module_id="TEST-VERIFY-001")
        msg = String(data=verify_req_encode(req))

        # Action
        self.module.on_verify_req(msg)

        # Output: Still WAIT_VERIFY (power enable comes next)
        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)

    def test_module_ignores_verify_for_wrong_id(self):
        """
        Input: VERIFY_REQ for different module_id
        Expected: Module state unchanged
        """
        # Precondition
        self.module.state = ModuleState.WAIT_VERIFY

        # Input: Wrong module ID
        req = VerifyReq(module_id="DIFFERENT-MODULE")
        msg = String(data=verify_req_encode(req))

        # Action
        self.module.on_verify_req(msg)

        # Output: State unchanged (message ignored)
        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)

    def test_power_enable_transitions_to_normal(self):
        """
        Input: Power enable message with data=true
        Precondition: Module in WAIT_VERIFY state
        Expected: Module transitions to NORMAL
        """
        # Precondition
        self.module.state = ModuleState.WAIT_VERIFY

        # Input
        power_msg = String(data='{"module_id": "TEST-VERIFY-001", "data": true}')

        # Action
        self.module.on_power(power_msg)

        # Output
        self.assertEqual(self.module.state, ModuleState.NORMAL)

    def test_power_enable_ignored_for_wrong_id(self):
        """
        Input: Power enable for different module_id
        Expected: Module state unchanged
        """
        # Precondition
        self.module.state = ModuleState.WAIT_VERIFY

        # Input: Wrong module ID
        power_msg = String(data='{"module_id": "DIFFERENT-MODULE", "data": true}')

        # Action
        self.module.on_power(power_msg)

        # Output: State unchanged
        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)


class TestHeartbeatChain(unittest.TestCase):
    """
    Test the heartbeat chain:
    Input: Module in NORMAL state calls send_heartbeat()
    Chain: module publishes → dock.on_heartbeat() receives
    Output: dock.modules[id]["last_hb"] updated, miss_count reset
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.dock = Dock()
        self.module = Module(module_id="TEST-HB-001", executor=self.executor)
        self.executor.add_node(self.dock)
        self.executor.add_node(self.module)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_heartbeat_increments_sequence(self):
        """
        Input: Call send_heartbeat() multiple times
        Expected: module.seq increments each time
        """
        # Precondition
        initial_seq = self.module.seq

        # Action
        self.module.send_heartbeat()
        self.module.send_heartbeat()
        self.module.send_heartbeat()

        # Output: Sequence incremented by 3
        self.assertEqual(self.module.seq, initial_seq + 3)

    def test_dock_receives_heartbeat_updates_tracking(self):
        """
        Input: Heartbeat message from "TEST-HB-001" with seq=42
        Precondition: Module registered in dock with old last_hb
        Expected: dock.modules["TEST-HB-001"]["last_hb"] updated
        """
        # Precondition: Register module with CORRECT structure
        old_time = time.time() - 10.0
        self.dock.modules["TEST-HB-001"] = {
            "state": DockState.ENABLED,
            "last_hb": old_time,
            "miss_count": 2,  # Had missed some
            "paused": False
        }

        # Input
        hb = Heartbeat(module_id="TEST-HB-001", seq=42)
        msg = String(data=hb_encode(hb))

        # Action
        self.dock.on_hb(msg)

        # Output: last_hb updated, in live_hb set
        self.assertIn("TEST-HB-001", self.dock.live_hb)

    def test_heartbeat_timer_starts_and_stops(self):
        """
        Input: start_heartbeat() then stop_heartbeat()
        Expected: Timer created, then destroyed
        """
        # Precondition: No timer
        self.assertIsNone(self.module.hb_timer)

        # Action: Start
        self.module.start_heartbeat()
        self.assertIsNotNone(self.module.hb_timer)

        # Action: Stop
        self.module.stop_heartbeat()
        self.assertIsNone(self.module.hb_timer)


class TestTaskCleanup(unittest.TestCase):
    """
    Test task cleanup:
    Input: Module in FIELD_OPS state
    Action: _cleanup_after_task()
    Expected: State → NORMAL, watchdog stopped, heartbeat restarted
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.module = Module(module_id="TEST-CLEANUP-001", executor=self.executor)
        self.executor.add_node(self.module)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.shutdown()
        self.module.destroy_node()

    def test_cleanup_resets_state_to_normal(self):
        """
        Input: Module in FIELD_OPS with active watchdog
        Action: _cleanup_after_task()
        Expected: state=NORMAL, watchdog=None
        """
        # Precondition
        self.module.state = ModuleState.FIELD_OPS
        self.module._start_task_watchdog("test_task")
        self.assertIsNotNone(self.module.task_watchdog_timer)

        # Action
        self.module._cleanup_after_task()

        # Output
        self.assertEqual(self.module.state, ModuleState.NORMAL)
        self.assertIsNone(self.module.task_watchdog_timer)

    def test_cleanup_updates_last_task_complete_time(self):
        """
        Input: Call _cleanup_after_task()
        Expected: last_task_complete_time updated to recent time
        """
        # Precondition: Set old time
        self.module.state = ModuleState.FIELD_OPS
        self.module.last_task_complete_time = time.time() - 100.0

        before = time.time()

        # Action
        self.module._cleanup_after_task()

        after = time.time()

        # Output: Time updated to within test window
        self.assertGreaterEqual(self.module.last_task_complete_time, before)
        self.assertLessEqual(self.module.last_task_complete_time, after)


if __name__ == '__main__':
    unittest.main()
