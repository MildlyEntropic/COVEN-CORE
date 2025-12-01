"""
test_fsm_transitions.py — FSM State Transition Tests for COVEN

Tests each state transition with:
- Input: message or action
- Precondition: current state
- Expected: new state

Every test follows: known input → expected output

Author: Alexander Shultis
Date: November 2025
"""

import unittest
import time
from std_msgs.msg import String
import rclpy
from rclpy.executors import SingleThreadedExecutor

from coven_core.module_node import Module
from coven_core.dock_node import Dock
from coven_core.common import (
    ModuleState, DockState,
    IdentifyReq, IdentifyRep, VerifyReq, VerifyRep,
    ident_req_encode, ident_rep_encode,
    verify_req_encode, verify_rep_encode,
)


class TestModuleStateTransitions(unittest.TestCase):
    """Test Module FSM transitions: input → expected state."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.module = Module(module_id="FSM-001", executor=self.executor)
        self.executor.add_node(self.module)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.remove_node(self.module)
        self.module.destroy_node()

    def test_initial_state_is_boot(self):
        """New module starts in BOOT state."""
        self.assertEqual(self.module.state, ModuleState.BOOT)

    def test_boot_to_wait_verify_on_identify(self):
        """
        Input: IDENTIFY_REQ
        Precondition: BOOT
        Expected: WAIT_VERIFY
        """
        self.assertEqual(self.module.state, ModuleState.BOOT)

        req = IdentifyReq(req_id="fsm-test")
        msg = String(data=ident_req_encode(req))
        self.module.on_ident_req(msg)

        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)

    def test_wait_verify_stays_on_verify_req(self):
        """
        Input: VERIFY_REQ (matching module_id)
        Precondition: WAIT_VERIFY
        Expected: Still WAIT_VERIFY (waiting for power enable)
        """
        self.module.state = ModuleState.WAIT_VERIFY

        req = VerifyReq(module_id="FSM-001")
        msg = String(data=verify_req_encode(req))
        self.module.on_verify_req(msg)

        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)

    def test_wait_verify_to_normal_on_power_enable(self):
        """
        Input: Power enable message (data=true)
        Precondition: WAIT_VERIFY
        Expected: NORMAL
        """
        self.module.state = ModuleState.WAIT_VERIFY

        power_msg = String(data='{"module_id": "FSM-001", "data": true}')
        self.module.on_power(power_msg)

        self.assertEqual(self.module.state, ModuleState.NORMAL)

    def test_normal_stays_on_identify(self):
        """
        Input: IDENTIFY_REQ
        Precondition: NORMAL
        Expected: Still NORMAL (ignores identify when not in BOOT)
        """
        self.module.state = ModuleState.NORMAL

        req = IdentifyReq(req_id="should-ignore")
        msg = String(data=ident_req_encode(req))
        self.module.on_ident_req(msg)

        self.assertEqual(self.module.state, ModuleState.NORMAL)

    def test_field_ops_to_normal_on_cleanup(self):
        """
        Input: _cleanup_after_task()
        Precondition: FIELD_OPS
        Expected: NORMAL
        """
        self.module.state = ModuleState.FIELD_OPS

        self.module._cleanup_after_task()

        self.assertEqual(self.module.state, ModuleState.NORMAL)


class TestModuleHeartbeatControl(unittest.TestCase):
    """Test heartbeat timer control: start/stop behavior."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.module = Module(module_id="HB-FSM-001", executor=self.executor)
        self.executor.add_node(self.module)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.remove_node(self.module)
        self.module.destroy_node()

    def test_heartbeat_timer_none_initially(self):
        """New module has no heartbeat timer."""
        self.assertIsNone(self.module.hb_timer)

    def test_start_heartbeat_creates_timer(self):
        """
        Input: start_heartbeat()
        Precondition: hb_timer is None
        Expected: hb_timer is not None
        """
        self.assertIsNone(self.module.hb_timer)

        self.module.start_heartbeat()

        self.assertIsNotNone(self.module.hb_timer)

    def test_stop_heartbeat_destroys_timer(self):
        """
        Input: stop_heartbeat()
        Precondition: hb_timer is active
        Expected: hb_timer is None
        """
        self.module.start_heartbeat()
        self.assertIsNotNone(self.module.hb_timer)

        self.module.stop_heartbeat()

        self.assertIsNone(self.module.hb_timer)

    def test_start_heartbeat_is_idempotent(self):
        """
        Input: start_heartbeat() called twice
        Expected: Does not create duplicate timer
        """
        self.module.start_heartbeat()
        timer1 = self.module.hb_timer

        self.module.start_heartbeat()
        timer2 = self.module.hb_timer

        # Same timer object (not a new one)
        self.assertIs(timer1, timer2)


class TestDockModuleTracking(unittest.TestCase):
    """Test dock module registration and state tracking."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.dock = Dock()
        self.executor.add_node(self.dock)

    def tearDown(self):
        self.executor.remove_node(self.dock)
        self.dock.destroy_node()

    def test_dock_starts_with_no_modules(self):
        """New dock has empty modules dict."""
        self.assertEqual(len(self.dock.modules), 0)

    def test_ident_rep_registers_module_in_verify_state(self):
        """
        Input: IDENTIFY_REP from module "DOCK-TEST-001"
        Expected: dock.modules["DOCK-TEST-001"]["state"] == VERIFY
        """
        rep = IdentifyRep(
            req_id="test",
            module_id="DOCK-TEST-001",
            module_type="ReconRover",
            fw="1.0.0"
        )
        msg = String(data=ident_rep_encode(rep))

        self.dock.on_ident_rep(msg)

        self.assertIn("DOCK-TEST-001", self.dock.modules)
        self.assertEqual(self.dock.modules["DOCK-TEST-001"]["state"], DockState.VERIFY)

    def test_verify_rep_ok_transitions_to_enabled(self):
        """
        Input: VERIFY_REP with ok=True
        Precondition: Module in VERIFY state
        Expected: Module state → ENABLED
        """
        # Register module first
        self.dock.modules["DOCK-TEST-002"] = {
            "state": DockState.VERIFY,
            "last_hb": time.time(),
            "miss_count": 0,
            "paused": False
        }

        rep = VerifyRep(module_id="DOCK-TEST-002", ok=True, reason="")
        msg = String(data=verify_rep_encode(rep))

        self.dock.on_verify_rep(msg)

        self.assertEqual(self.dock.modules["DOCK-TEST-002"]["state"], DockState.ENABLED)

    def test_verify_rep_fail_transitions_to_rejected(self):
        """
        Input: VERIFY_REP with ok=False
        Precondition: Module in VERIFY state
        Expected: Module state → REJECTED
        """
        # Register module first
        self.dock.modules["DOCK-TEST-003"] = {
            "state": DockState.VERIFY,
            "last_hb": time.time(),
            "miss_count": 0,
            "paused": False
        }

        rep = VerifyRep(module_id="DOCK-TEST-003", ok=False, reason="Health check failed")
        msg = String(data=verify_rep_encode(rep))

        self.dock.on_verify_rep(msg)

        self.assertEqual(self.dock.modules["DOCK-TEST-003"]["state"], DockState.REJECTED)

    def test_verify_rep_for_unknown_module_ignored(self):
        """
        Input: VERIFY_REP for module not in dock.modules
        Expected: No crash, no change
        """
        initial_count = len(self.dock.modules)

        rep = VerifyRep(module_id="UNKNOWN-MODULE", ok=True, reason="")
        msg = String(data=verify_rep_encode(rep))

        # Should not crash
        self.dock.on_verify_rep(msg)

        # No new modules added
        self.assertEqual(len(self.dock.modules), initial_count)


class TestWatchdogTimer(unittest.TestCase):
    """Test task watchdog timer behavior."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.module = Module(module_id="WD-001", executor=self.executor)
        self.executor.add_node(self.module)

    def tearDown(self):
        if self.module.task_watchdog_timer:
            self.module._stop_task_watchdog()
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.remove_node(self.module)
        self.module.destroy_node()

    def test_watchdog_none_initially(self):
        """New module has no watchdog timer."""
        self.assertIsNone(self.module.task_watchdog_timer)

    def test_start_watchdog_creates_timer(self):
        """
        Input: _start_task_watchdog("test")
        Expected: task_watchdog_timer is not None
        """
        self.module._start_task_watchdog("test_task")

        self.assertIsNotNone(self.module.task_watchdog_timer)

    def test_stop_watchdog_destroys_timer(self):
        """
        Input: _stop_task_watchdog()
        Precondition: Watchdog active
        Expected: task_watchdog_timer is None
        """
        self.module._start_task_watchdog("test_task")
        self.assertIsNotNone(self.module.task_watchdog_timer)

        self.module._stop_task_watchdog()

        self.assertIsNone(self.module.task_watchdog_timer)

    def test_cleanup_stops_watchdog(self):
        """
        Input: _cleanup_after_task()
        Precondition: Watchdog active
        Expected: Watchdog stopped
        """
        self.module.state = ModuleState.FIELD_OPS
        self.module._start_task_watchdog("test_task")

        self.module._cleanup_after_task()

        self.assertIsNone(self.module.task_watchdog_timer)


if __name__ == '__main__':
    unittest.main()
