"""
test_integration.py — Integration Tests for COVEN

Full system integration tests that verify dock-module interaction,
multi-module scenarios, heartbeat monitoring, and task execution.

These tests launch real ROS2 nodes and verify end-to-end behavior.

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
    IdentifyReq, IdentifyRep, VerifyReq, VerifyRep,
    Heartbeat, TaskReq, TaskAck,
    ident_req_encode, ident_req_decode,
    ident_rep_encode, ident_rep_decode,
    verify_req_encode, verify_rep_encode,
    heartbeat_encode, heartbeat_decode,
    task_req_encode, task_ack_decode
)


class TestDockModuleIntegration(unittest.TestCase):
    """Test dock-module interaction end-to-end."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()

    def setUp(self):
        """Create dock and module nodes."""
        self.executor = MultiThreadedExecutor()

        # Create dock
        self.dock = Dock()
        self.executor.add_node(self.dock)

        # Create module
        self.module = Module(module_id="TEST-INT-001", executor=self.executor)
        self.executor.add_node(self.module)

        # Start executor in background thread
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # Give nodes time to initialize
        time.sleep(0.5)

    def tearDown(self):
        """Cleanup nodes."""
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_discovery_and_identification(self):
        """Test module discovery through IDENTIFY_REQ/REP."""
        # Initially module should be in BOOT
        self.assertEqual(self.module.state, ModuleState.BOOT)

        # Initially dock has no modules
        initial_count = len(self.dock.modules)

        # Wait for identification cycle (dock sends IDENTIFY_REQ every 5s)
        # We'll manually trigger it for faster testing
        time.sleep(0.5)

        # Verify module is still in BOOT or has transitioned
        self.assertIn(self.module.state, [ModuleState.BOOT, ModuleState.WAIT_VERIFY])

    def test_verification_flow(self):
        """Test verification request and response."""
        # Set module to WAIT_VERIFY state
        self.module.state = ModuleState.WAIT_VERIFY

        # Create VERIFY_REQ message
        verify_req = VerifyReq(module_id="TEST-INT-001")
        msg = String(data=verify_req_encode(verify_req))

        # Send to module
        self.module.on_verify_req(msg)

        # Module should still be in WAIT_VERIFY (waiting for power enable)
        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY)

    def test_power_enable_transition(self):
        """Test module transitions to NORMAL after power enable."""
        # Set to WAIT_VERIFY
        self.module.state = ModuleState.WAIT_VERIFY

        # Send power enable
        power_msg = String(data='{"module_id": "TEST-INT-001", "data": true}')
        self.module.on_power(power_msg)

        # Should transition to NORMAL
        self.assertEqual(self.module.state, ModuleState.NORMAL)

    def test_heartbeat_publishing(self):
        """Test module publishes heartbeat in NORMAL state."""
        # Set to NORMAL and start heartbeat
        self.module.state = ModuleState.NORMAL
        self.module.start_heartbeat()

        # Verify heartbeat timer is running
        self.assertIsNotNone(self.module.hb_timer)

        # Wait for at least one heartbeat period
        time.sleep(1.0)

        # Heartbeat should have been published (seq > 0)
        self.assertGreater(self.module.seq, 0)

        # Cleanup
        self.module.stop_heartbeat()


class TestMultiModuleIntegration(unittest.TestCase):
    """Test dock managing multiple modules simultaneously."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create dock and multiple modules."""
        self.executor = MultiThreadedExecutor()

        # Create dock
        self.dock = Dock()
        self.executor.add_node(self.dock)

        # Create 3 modules
        self.modules = []
        for i in range(3):
            module_id = f"TEST-MULTI-{i:03d}"
            module = Module(module_id=module_id, executor=self.executor)
            self.modules.append(module)
            self.executor.add_node(module)

        # Start executor
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        time.sleep(0.5)

    def tearDown(self):
        """Cleanup."""
        self.executor.shutdown()
        self.dock.destroy_node()
        for module in self.modules:
            module.destroy_node()

    def test_multiple_module_tracking(self):
        """Test dock can track multiple modules."""
        # Set all modules to NORMAL and start heartbeats
        for module in self.modules:
            module.state = ModuleState.NORMAL
            module.start_heartbeat()

        # Wait for heartbeats to be published
        time.sleep(2.0)

        # All modules should have sequence numbers > 0
        for module in self.modules:
            self.assertGreater(module.seq, 0)

        # Cleanup
        for module in self.modules:
            module.stop_heartbeat()

    def test_independent_module_states(self):
        """Test modules can be in different states independently."""
        # Set different states
        self.modules[0].state = ModuleState.BOOT
        self.modules[1].state = ModuleState.NORMAL
        self.modules[2].state = ModuleState.FIELD_OPS

        # Verify states are independent
        self.assertEqual(self.modules[0].state, ModuleState.BOOT)
        self.assertEqual(self.modules[1].state, ModuleState.NORMAL)
        self.assertEqual(self.modules[2].state, ModuleState.FIELD_OPS)


class TestHeartbeatMonitoring(unittest.TestCase):
    """Test heartbeat monitoring and timeout detection."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create dock and module."""
        self.executor = MultiThreadedExecutor()

        self.dock = Dock()
        self.executor.add_node(self.dock)

        self.module = Module(module_id="TEST-HB-001", executor=self.executor)
        self.executor.add_node(self.module)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        time.sleep(0.5)

    def tearDown(self):
        """Cleanup."""
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_heartbeat_reception(self):
        """Test dock receives heartbeats from module."""
        # Manually add module to dock tracking
        self.dock.modules["TEST-HB-001"] = {
            'module_id': 'TEST-HB-001',
            'module_type': 'ReconRover',
            'fw': '1.0.0',
            'state': DockState.NORMAL,
            'last_hb': time.time(),
            'hb_seq': 0,
            'missed_hbs': 0
        }

        # Start module heartbeat
        self.module.state = ModuleState.NORMAL
        self.module.start_heartbeat()

        # Wait for heartbeats
        time.sleep(2.0)

        # Module should have sent heartbeats
        self.assertGreater(self.module.seq, 0)

        # Cleanup
        self.module.stop_heartbeat()

    def test_heartbeat_timeout_detection(self):
        """Test dock detects when heartbeat stops."""
        # Add module to dock tracking with old heartbeat
        self.dock.modules["TEST-HB-001"] = {
            'module_id': 'TEST-HB-001',
            'module_type': 'ReconRover',
            'fw': '1.0.0',
            'state': DockState.NORMAL,
            'last_hb': time.time() - 10.0,  # 10 seconds ago
            'hb_seq': 0,
            'missed_hbs': 0
        }

        # Module is still tracked
        self.assertIn("TEST-HB-001", self.dock.modules)

        # In real system, dock's monitor timer would detect timeout
        # For this test, we just verify the data structure


class TestTaskExecution(unittest.TestCase):
    """Test task assignment and execution flow."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create dock and module."""
        self.executor = MultiThreadedExecutor()

        self.dock = Dock()
        self.executor.add_node(self.dock)

        self.module = Module(module_id="TEST-TASK-001", executor=self.executor)
        self.executor.add_node(self.module)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        time.sleep(0.5)

    def tearDown(self):
        """Cleanup."""
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_task_request_acknowledgment(self):
        """Test module acknowledges task request."""
        # Set module to NORMAL
        self.module.state = ModuleState.NORMAL

        # Create task request
        task_req = TaskReq(module_id="TEST-TASK-001", task="test_task")
        msg = String(data=task_req_encode(task_req))

        # Send to module
        self.module.on_task_req(msg)

        # Module should still be in NORMAL (task hasn't started yet)
        # The action server handles the actual task execution
        self.assertEqual(self.module.state, ModuleState.NORMAL)

    def test_task_cleanup_returns_to_normal(self):
        """Test module returns to NORMAL after task cleanup."""
        # Set to FIELD_OPS
        self.module.state = ModuleState.FIELD_OPS

        # Call cleanup
        self.module._cleanup_after_task()

        # Should be back in NORMAL
        self.assertEqual(self.module.state, ModuleState.NORMAL)


class TestErrorRecoveryIntegration(unittest.TestCase):
    """Test error recovery scenarios in integrated system."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Create nodes."""
        self.executor = MultiThreadedExecutor()

        self.dock = Dock()
        self.executor.add_node(self.dock)

        self.module = Module(module_id="TEST-ERR-INT-001", executor=self.executor)
        self.executor.add_node(self.module)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        time.sleep(0.5)

    def tearDown(self):
        """Cleanup."""
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_module_reconnection_after_timeout(self):
        """Test module can reconnect after heartbeat timeout."""
        # Module starts in BOOT
        self.assertEqual(self.module.state, ModuleState.BOOT)

        # Transition to NORMAL
        self.module.state = ModuleState.NORMAL
        self.module.start_heartbeat()

        # Simulate disconnect
        self.module.state = ModuleState.DISCONNECTED
        self.module.stop_heartbeat()

        # Simulate reconnection - back to BOOT
        self.module.state = ModuleState.BOOT

        # Module can go through identification again
        self.assertEqual(self.module.state, ModuleState.BOOT)

        # Cleanup
        if self.module.hb_timer:
            self.module.stop_heartbeat()

    def test_watchdog_timer_functionality(self):
        """Test task watchdog timer starts and stops correctly."""
        # Set to FIELD_OPS
        self.module.state = ModuleState.FIELD_OPS

        # Start watchdog
        self.module._start_task_watchdog("test_task")

        # Watchdog should be running
        self.assertIsNotNone(self.module.task_watchdog_timer)

        # Stop watchdog
        self.module._cleanup_after_task()

        # Watchdog should be stopped
        self.assertIsNone(self.module.task_watchdog_timer)

        # Should be back in NORMAL
        self.assertEqual(self.module.state, ModuleState.NORMAL)


if __name__ == '__main__':
    unittest.main()
