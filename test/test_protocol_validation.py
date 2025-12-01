"""
test_protocol_validation.py — COVEN Protocol Validation Tests

These tests validate the COMPLETE protocol flows end-to-end:
- Full discovery handshake
- Complete verification sequence
- Task assignment and acknowledgment
- Error handling and recovery

This is not "did the function run?" but "did the PROTOCOL work correctly?"

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
    IdentifyReq, ident_req_encode,
    ident_rep_decode,
    verify_rep_decode,
)


class ProtocolValidator:
    """Helper to validate protocol state machine progressions."""

    def __init__(self):
        self.events = []
        self.lock = threading.Lock()

    def record(self, event_type, **kwargs):
        with self.lock:
            self.events.append({
                'type': event_type,
                'timestamp': time.time(),
                **kwargs
            })

    def get_sequence(self):
        with self.lock:
            return [e['type'] for e in self.events]

    def validate_sequence(self, expected_sequence):
        """Validate events occurred in expected order."""
        actual = self.get_sequence()

        # Check all expected events occurred
        for expected_event in expected_sequence:
            if expected_event not in actual:
                return False, f"Missing event: {expected_event}"

        # Check order
        expected_indices = []
        for expected_event in expected_sequence:
            try:
                idx = actual.index(expected_event)
                expected_indices.append((expected_event, idx))
            except ValueError:
                return False, f"Event not found: {expected_event}"

        # Verify ordering
        for i in range(len(expected_indices) - 1):
            if expected_indices[i][1] > expected_indices[i+1][1]:
                return False, f"Out of order: {expected_indices[i][0]} after {expected_indices[i+1][0]}"

        return True, f"Sequence valid: {' → '.join(expected_sequence)}"

    def get_timing(self, event1, event2):
        """Get time between two events."""
        with self.lock:
            time1 = next((e['timestamp'] for e in self.events if e['type'] == event1), None)
            time2 = next((e['timestamp'] for e in self.events if e['type'] == event2), None)

            if time1 and time2:
                return time2 - time1
            return None


class TestCompleteDiscoveryProtocol(unittest.TestCase):
    """
    Test the COMPLETE discovery protocol end-to-end.

    Protocol:
    1. Dock broadcasts IDENTIFY_REQ
    2. Module (in BOOT) receives and responds IDENTIFY_REP
    3. Module transitions BOOT → WAIT_VERIFY
    4. Dock receives IDENTIFY_REP and records module
    5. Dock sends VERIFY_REQ to specific module
    6. Module responds VERIFY_REP
    7. If OK, Dock sends power enable
    8. Module transitions WAIT_VERIFY → NORMAL
    9. Module starts heartbeat
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        pass  # Don't shutdown - other test classes may need rclpy

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.validator = ProtocolValidator()

        # Create dock and module
        self.dock = Dock()
        self.module = Module(module_id="PROTO-001", executor=self.executor)

        self.executor.add_node(self.dock)
        self.executor.add_node(self.module)

        # Subscribe to track protocol messages
        self.ident_rep_sub = self.dock.create_subscription(
            String,
            '/coven/identify_rep',
            lambda msg: self.validator.record('IDENTIFY_REP_RECEIVED', data=ident_rep_decode(msg)),
            10
        )

        self.verify_rep_sub = self.dock.create_subscription(
            String,
            '/coven/verify_rep',
            lambda msg: self.validator.record('VERIFY_REP_RECEIVED', data=verify_rep_decode(msg)),
            10
        )

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_full_discovery_sequence(self):
        """
        Test complete discovery sequence with timing requirements.

        Requirements:
        - Module must be in BOOT initially
        - Module must respond to IDENTIFY_REQ within 100ms
        - Module must transition to WAIT_VERIFY after responding
        - Dock must receive IDENTIFY_REP
        """
        # 1. Verify initial state
        self.assertEqual(self.module.state, ModuleState.BOOT,
                         "Module must start in BOOT state")
        self.validator.record('MODULE_BOOT')

        # 2. Dock broadcasts IDENTIFY_REQ
        req = IdentifyReq(req_id="full-test")
        msg = String(data=ident_req_encode(req))

        start_time = time.time()
        self.validator.record('IDENTIFY_REQ_SENT')

        # Dock would normally broadcast, we'll send directly to module
        self.module.on_ident_req(msg)

        # 3. Wait for response
        time.sleep(0.5)

        # 4. Validate response was received
        events = self.validator.get_sequence()
        self.assertIn('IDENTIFY_REP_RECEIVED', events,
                      "Dock did not receive IDENTIFY_REP")

        # 5. Check response timing
        response_time = self.validator.get_timing('IDENTIFY_REQ_SENT', 'IDENTIFY_REP_RECEIVED')
        self.assertIsNotNone(response_time)
        self.assertLess(response_time, 0.1,
                        f"Response time {response_time*1000:.1f}ms exceeds 100ms requirement")

        # 6. Verify state transition
        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY,
                         "Module must transition to WAIT_VERIFY after identification")

        # 7. Validate protocol sequence
        valid, msg = self.validator.validate_sequence([
            'MODULE_BOOT',
            'IDENTIFY_REQ_SENT',
            'IDENTIFY_REP_RECEIVED'
        ])
        self.assertTrue(valid, f"Protocol sequence invalid: {msg}")

        print(f"✓ Discovery protocol complete in {response_time*1000:.1f}ms")
        print(f"✓ State progression: BOOT → WAIT_VERIFY")

    def test_verification_protocol(self):
        """
        Test complete verification protocol.

        Requirements:
        - Module in WAIT_VERIFY must respond to VERIFY_REQ
        - Module must report health check status
        - Verification response must be received by dock
        """
        # Setup: module in WAIT_VERIFY (skip identification)
        self.module.state = ModuleState.WAIT_VERIFY

        # Send VERIFY_REQ
        from coven_core.common import VerifyReq, verify_req_encode
        req = VerifyReq(module_id="PROTO-001")
        msg = String(data=verify_req_encode(req))

        self.validator.record('VERIFY_REQ_SENT')
        self.module.on_verify_req(msg)

        time.sleep(0.5)

        # Check response received
        events = self.validator.get_sequence()
        self.assertIn('VERIFY_REP_RECEIVED', events,
                      "Dock did not receive VERIFY_REP")

        # Validate response data
        verify_events = [e for e in self.validator.events if e['type'] == 'VERIFY_REP_RECEIVED']
        self.assertEqual(len(verify_events), 1, "Should receive exactly one VERIFY_REP")

        verify_data = verify_events[0]['data']
        self.assertIsNotNone(verify_data)
        self.assertEqual(verify_data.module_id, "PROTO-001")
        self.assertIsInstance(verify_data.ok, bool)
        self.assertIsInstance(verify_data.reason, str)

        print(f"✓ Verification: ok={verify_data.ok}, reason='{verify_data.reason}'")

    def test_power_enable_to_heartbeat(self):
        """
        Test complete power enable → heartbeat sequence.

        Requirements:
        - Module in WAIT_VERIFY
        - Power enable message transitions to NORMAL
        - Heartbeat starts within 1 second
        - Heartbeat contains correct module_id
        """
        # Setup
        self.module.state = ModuleState.WAIT_VERIFY

        # Track heartbeats
        hb_count = [0]

        def hb_callback(msg):
            from coven_core.common import hb_decode
            hb = hb_decode(msg)
            if hb and hb.module_id == "PROTO-001":
                hb_count[0] += 1
                self.validator.record('HEARTBEAT_RECEIVED', seq=hb.seq)

        hb_sub = self.dock.create_subscription(String, '/coven/heartbeat', hb_callback, 10)

        # Send power enable
        power_msg = String(data='{"module_id": "PROTO-001", "data": true}')
        self.validator.record('POWER_ENABLE_SENT')
        self.module.on_power(power_msg)

        # Module should transition to NORMAL
        self.assertEqual(self.module.state, ModuleState.NORMAL,
                         "Module must transition to NORMAL after power enable")

        # Wait for heartbeats
        time.sleep(2.0)

        # Check heartbeats started
        self.assertGreater(hb_count[0], 0,
                           "Module did not start sending heartbeats in NORMAL state")

        # Validate timing
        hb_start_time = self.validator.get_timing('POWER_ENABLE_SENT', 'HEARTBEAT_RECEIVED')
        self.assertIsNotNone(hb_start_time)
        self.assertLess(hb_start_time, 1.0,
                        f"Heartbeat started {hb_start_time:.2f}s after power enable (requirement: <1s)")

        print(f"✓ Power enable → Heartbeat in {hb_start_time:.3f}s")
        print(f"✓ Heartbeats received: {hb_count[0]}")

        # Cleanup
        self.module.stop_heartbeat()


class TestFailureProtocols(unittest.TestCase):
    """Test protocol behavior under failure conditions."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def test_module_ignores_identify_when_not_in_boot(self):
        """
        Test module correctly ignores IDENTIFY_REQ when not in BOOT.

        Requirement: Module in NORMAL should not respond to IDENTIFY_REQ
        """
        executor = MultiThreadedExecutor()
        module = Module(module_id="FAIL-001", executor=executor)
        executor.add_node(module)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        time.sleep(0.5)

        # Set module to NORMAL (not BOOT)
        module.state = ModuleState.NORMAL

        # Track responses
        response_count = [0]

        def ident_callback(msg):
            response_count[0] += 1

        sub = module.create_subscription(String, '/coven/identify_rep', ident_callback, 10)

        # Send IDENTIFY_REQ
        req = IdentifyReq(req_id="should-ignore")
        msg = String(data=ident_req_encode(req))
        module.on_ident_req(msg)

        time.sleep(0.5)

        # Should NOT respond
        self.assertEqual(response_count[0], 0,
                         "Module incorrectly responded to IDENTIFY_REQ while in NORMAL state")

        executor.shutdown()
        module.destroy_node()

        print("✓ Module correctly ignores IDENTIFY_REQ when not in BOOT")

    def test_module_ignores_verify_for_wrong_id(self):
        """
        Test module ignores VERIFY_REQ for different module_id.

        Requirement: Module must only respond to its own module_id
        """
        executor = MultiThreadedExecutor()
        module = Module(module_id="FAIL-002", executor=executor)
        executor.add_node(module)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        time.sleep(0.5)

        module.state = ModuleState.WAIT_VERIFY

        response_count = [0]

        def verify_callback(msg):
            response_count[0] += 1

        sub = module.create_subscription(String, '/coven/verify_rep', verify_callback, 10)

        # Send VERIFY_REQ for DIFFERENT module
        from coven_core.common import VerifyReq, verify_req_encode
        req = VerifyReq(module_id="DIFFERENT-MODULE")
        msg = String(data=verify_req_encode(req))
        module.on_verify_req(msg)

        time.sleep(0.5)

        # Should NOT respond
        self.assertEqual(response_count[0], 0,
                         "Module incorrectly responded to VERIFY_REQ for different module_id")

        executor.shutdown()
        module.destroy_node()

        print("✓ Module correctly ignores VERIFY_REQ for wrong module_id")


class TestProtocolMetrics(unittest.TestCase):
    """Test quantitative protocol metrics."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def test_full_boot_to_heartbeat_latency(self):
        """
        Measure complete boot → heartbeat latency.

        This is the REAL test: How long from power-on to operational?
        Requirement: < 3 seconds
        """
        executor = MultiThreadedExecutor()

        start_time = time.time()

        # Create module (simulates power-on)
        module = Module(module_id="METRIC-001", executor=executor)
        executor.add_node(module)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Track first heartbeat
        first_hb_time = [None]

        def hb_callback(msg):
            if first_hb_time[0] is None:
                from coven_core.common import hb_decode
                hb = hb_decode(msg)
                if hb and hb.module_id == "METRIC-001":
                    first_hb_time[0] = time.time()

        hb_sub = module.create_subscription(String, '/coven/heartbeat', hb_callback, 10)

        # Simulate discovery and verification
        time.sleep(0.5)

        # Identification
        req = IdentifyReq(req_id="metric-test")
        msg = String(data=ident_req_encode(req))
        module.on_ident_req(msg)

        time.sleep(0.2)

        # Verification
        from coven_core.common import VerifyReq, verify_req_encode
        verify_msg = String(data=verify_req_encode(VerifyReq(module_id="METRIC-001")))
        module.on_verify_req(verify_msg)

        time.sleep(0.2)

        # Power enable
        power_msg = String(data='{"module_id": "METRIC-001", "data": true}')
        module.on_power(power_msg)

        # Wait for first heartbeat
        timeout = 5.0
        elapsed = 0
        while first_hb_time[0] is None and elapsed < timeout:
            time.sleep(0.1)
            elapsed += 0.1

        executor.shutdown()
        module.destroy_node()

        # Calculate total latency
        self.assertIsNotNone(first_hb_time[0], "No heartbeat received")

        total_latency = first_hb_time[0] - start_time

        self.assertLess(total_latency, 3.0,
                        f"Boot-to-heartbeat latency {total_latency:.2f}s exceeds 3s requirement")

        print(f"✓ Complete boot-to-heartbeat latency: {total_latency:.3f}s")
        print(f"  (Requirement: <3.0s, Measured: {total_latency:.3f}s)")


if __name__ == '__main__':
    unittest.main(verbosity=2)
