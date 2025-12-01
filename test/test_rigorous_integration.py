"""
test_rigorous_integration.py — Rigorous COVEN Integration Tests

These tests ACTUALLY validate the system with measurable criteria:
- Performance metrics (latency, throughput)
- Protocol correctness (full end-to-end validation)
- Failure mode handling
- Quantitative assertions with tolerances
- Real message delivery verification

NOT just "does timer exist?" but "does it work correctly and how well?"

Author: Alexander Shultis
Date: November 2025
"""

import unittest
import time
import threading
from collections import defaultdict
from std_msgs.msg import String
import rclpy
from rclpy.executors import MultiThreadedExecutor

from coven_core.module_node import Module
from coven_core.dock_node import Dock
from coven_core.common import (
    ModuleState, DockState,
    IdentifyReq, IdentifyRep, VerifyReq, VerifyRep,
    Heartbeat,
    ident_req_encode, ident_rep_encode, ident_rep_decode,
    verify_req_encode, verify_rep_encode,
    hb_encode, hb_decode,
)


class MessageCapture:
    """Helper class to capture and analyze ROS2 messages."""

    def __init__(self, node, topic, msg_type, decode_func=None):
        self.messages = []
        self.timestamps = []
        self.lock = threading.Lock()
        self.decode_func = decode_func

        self.subscription = node.create_subscription(
            msg_type,
            topic,
            self._callback,
            10
        )

    def _callback(self, msg):
        with self.lock:
            self.timestamps.append(time.time())
            if self.decode_func:
                decoded = self.decode_func(msg)
                self.messages.append(decoded)
            else:
                self.messages.append(msg)

    def get_count(self):
        with self.lock:
            return len(self.messages)

    def get_messages(self):
        with self.lock:
            return list(self.messages)

    def get_rate(self):
        """Calculate message rate in Hz."""
        with self.lock:
            if len(self.timestamps) < 2:
                return 0.0
            duration = self.timestamps[-1] - self.timestamps[0]
            return (len(self.timestamps) - 1) / duration if duration > 0 else 0.0

    def get_latencies(self):
        """Calculate inter-message latencies."""
        with self.lock:
            if len(self.timestamps) < 2:
                return []
            return [self.timestamps[i+1] - self.timestamps[i]
                    for i in range(len(self.timestamps)-1)]

    def get_timestamps(self):
        """Get all message timestamps."""
        with self.lock:
            return list(self.timestamps)

    def clear(self):
        with self.lock:
            self.messages.clear()
            self.timestamps.clear()


class TestRigorousIdentification(unittest.TestCase):
    """Rigorous tests for module identification protocol."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        pass  # Don't shutdown - other test classes may need rclpy

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.dock = Dock()
        self.module = Module(module_id="RIGOROUS-001", executor=self.executor)

        self.executor.add_node(self.dock)
        self.executor.add_node(self.module)

        # Capture IDENTIFY_REP messages
        self.ident_rep_capture = MessageCapture(
            self.dock,
            '/coven/identify_rep',
            String,
            ident_rep_decode
        )

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        self.executor.shutdown()
        self.dock.destroy_node()
        self.module.destroy_node()

    def test_identification_latency(self):
        """
        Test identification latency is acceptable.

        Requirement: Module must respond to IDENTIFY_REQ within 100ms
        """
        # Module should be in BOOT
        self.assertEqual(self.module.state, ModuleState.BOOT)

        # Clear any existing captures
        self.ident_rep_capture.clear()

        # Send IDENTIFY_REQ and measure latency
        start_time = time.time()

        req = IdentifyReq(req_id="latency-test")
        msg = String(data=ident_req_encode(req))
        self.module.on_ident_req(msg)

        # Wait for response (give enough time but measure actual latency)
        time.sleep(0.05)  # 50ms should be plenty

        # Check we got a response
        timestamps = self.ident_rep_capture.get_timestamps()
        self.assertGreater(len(timestamps), 0, "No IDENTIFY_REP received")

        # Calculate actual latency from message timestamp
        response_time = timestamps[0]
        latency_ms = (response_time - start_time) * 1000

        self.assertLess(latency_ms, 100,
                        f"Identification latency {latency_ms:.1f}ms exceeds 100ms requirement")

        print(f"✓ Identification latency: {latency_ms:.2f}ms")

    def test_identification_correctness(self):
        """
        Test identification response contains correct data.

        Requirement: IDENTIFY_REP must contain valid module_id, type, and firmware
        """
        req = IdentifyReq(req_id="correctness-test")
        msg = String(data=ident_req_encode(req))
        self.module.on_ident_req(msg)

        time.sleep(0.2)

        messages = self.ident_rep_capture.get_messages()
        self.assertEqual(len(messages), 1, "Expected exactly 1 IDENTIFY_REP")

        rep = messages[0]
        self.assertIsNotNone(rep)
        self.assertEqual(rep.req_id, "correctness-test")
        self.assertEqual(rep.module_id, "RIGOROUS-001")
        self.assertIsInstance(rep.module_type, str)
        self.assertGreater(len(rep.module_type), 0)
        self.assertIsInstance(rep.fw, str)
        self.assertGreater(len(rep.fw), 0)

        print(f"✓ Identification data: ID={rep.module_id}, Type={rep.module_type}, FW={rep.fw}")

    def test_identification_state_transition(self):
        """
        Test module transitions to WAIT_VERIFY after identification.

        Requirement: State machine must transition BOOT → WAIT_VERIFY
        """
        self.assertEqual(self.module.state, ModuleState.BOOT)

        req = IdentifyReq(req_id="state-test")
        msg = String(data=ident_req_encode(req))
        self.module.on_ident_req(msg)

        time.sleep(0.1)

        self.assertEqual(self.module.state, ModuleState.WAIT_VERIFY,
                         "Module did not transition to WAIT_VERIFY after identification")

        print("✓ State transition: BOOT → WAIT_VERIFY")


class TestRigorousHeartbeat(unittest.TestCase):
    """Rigorous tests for heartbeat monitoring."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.module = Module(module_id="HB-RIGOROUS", executor=self.executor)
        self.executor.add_node(self.module)

        # Capture heartbeats
        self.hb_capture = MessageCapture(
            self.module,
            '/coven/heartbeat',
            String,
            hb_decode
        )

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        if self.module.hb_timer:
            self.module.stop_heartbeat()
        self.executor.shutdown()
        self.module.destroy_node()

    def test_heartbeat_frequency(self):
        """
        Test heartbeat frequency matches configuration.

        Requirement: Heartbeats at configured rate (default ~1.25 Hz = 0.8s period)
        Tolerance: ±10%
        """
        # Configure heartbeat period
        expected_period = self.module.hb_period  # Default 0.8s
        expected_rate = 1.0 / expected_period  # ~1.25 Hz

        # Start heartbeat
        self.module.state = ModuleState.NORMAL
        self.module.start_heartbeat()

        # Collect heartbeats for 5 seconds
        time.sleep(5.0)

        # Measure actual rate
        actual_rate = self.hb_capture.get_rate()
        count = self.hb_capture.get_count()

        # Check we got heartbeats
        self.assertGreater(count, 4, f"Expected ~6 heartbeats in 5s, got {count}")

        # Check rate is within tolerance
        tolerance = 0.10  # 10%
        min_rate = expected_rate * (1 - tolerance)
        max_rate = expected_rate * (1 + tolerance)

        self.assertGreater(actual_rate, min_rate,
                           f"Heartbeat rate {actual_rate:.2f} Hz below minimum {min_rate:.2f} Hz")
        self.assertLess(actual_rate, max_rate,
                        f"Heartbeat rate {actual_rate:.2f} Hz above maximum {max_rate:.2f} Hz")

        print(f"✓ Heartbeat rate: {actual_rate:.2f} Hz (expected {expected_rate:.2f} Hz, ±10%)")
        print(f"✓ Heartbeats received: {count} in 5.0s")

    def test_heartbeat_consistency(self):
        """
        Test heartbeat timing consistency (jitter).

        Requirement: Inter-heartbeat latency variance < 20% of period
        """
        expected_period = self.module.hb_period

        self.module.state = ModuleState.NORMAL
        self.module.start_heartbeat()

        # Collect heartbeats
        time.sleep(5.0)

        latencies = self.hb_capture.get_latencies()
        self.assertGreater(len(latencies), 3, "Not enough heartbeats to measure consistency")

        # Calculate statistics
        avg_latency = sum(latencies) / len(latencies)
        max_deviation = max(abs(l - avg_latency) for l in latencies)
        jitter_percent = (max_deviation / expected_period) * 100

        # Check jitter is acceptable
        self.assertLess(jitter_percent, 20,
                        f"Heartbeat jitter {jitter_percent:.1f}% exceeds 20% limit")

        print(f"✓ Heartbeat timing: avg={avg_latency:.3f}s, jitter={jitter_percent:.1f}%")

    def test_heartbeat_sequence(self):
        """
        Test heartbeat sequence numbers increment correctly.

        Requirement: Sequence numbers must increment monotonically
        """
        self.module.state = ModuleState.NORMAL
        self.module.start_heartbeat()

        time.sleep(3.0)

        messages = self.hb_capture.get_messages()
        self.assertGreater(len(messages), 2, "Need at least 3 heartbeats")

        # Check sequence numbers increment
        for i in range(len(messages) - 1):
            self.assertGreater(messages[i+1].seq, messages[i].seq,
                               f"Sequence not incrementing: {messages[i].seq} → {messages[i+1].seq}")

        # Check no gaps (should increment by 1)
        for i in range(len(messages) - 1):
            diff = messages[i+1].seq - messages[i].seq
            self.assertEqual(diff, 1,
                             f"Sequence gap detected: {messages[i].seq} → {messages[i+1].seq}")

        print(f"✓ Sequence numbers: {messages[0].seq} → {messages[-1].seq} (monotonic)")


class TestRigorousMultiModule(unittest.TestCase):
    """Rigorous tests for multi-module coordination."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.executor = MultiThreadedExecutor()
        self.dock = Dock()
        self.executor.add_node(self.dock)

        # Create 5 modules
        self.modules = []
        for i in range(5):
            module = Module(module_id=f"MULTI-{i:03d}", executor=self.executor)
            self.modules.append(module)
            self.executor.add_node(module)

        # Capture heartbeats
        self.hb_capture = MessageCapture(
            self.dock,
            '/coven/heartbeat',
            String,
            hb_decode
        )

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.5)

    def tearDown(self):
        for module in self.modules:
            if module.hb_timer:
                module.stop_heartbeat()
        self.executor.shutdown()
        self.dock.destroy_node()
        for module in self.modules:
            module.destroy_node()

    def test_concurrent_heartbeats(self):
        """
        Test dock receives heartbeats from all modules concurrently.

        Requirement: All N modules sending heartbeats → dock receives N*rate messages
        """
        num_modules = len(self.modules)

        # Start all heartbeats
        for module in self.modules:
            module.state = ModuleState.NORMAL
            module.start_heartbeat()

        # Collect for 5 seconds
        time.sleep(5.0)

        messages = self.hb_capture.get_messages()
        total_count = len(messages)

        # Count per module
        module_counts = defaultdict(int)
        for msg in messages:
            if msg:
                module_counts[msg.module_id] += 1

        # Each module should send ~6 heartbeats in 5s (at 1.25 Hz)
        expected_per_module = 5  # Conservative
        expected_total = num_modules * expected_per_module

        self.assertGreater(total_count, expected_total * 0.8,
                           f"Expected ≥{expected_total * 0.8:.0f} heartbeats, got {total_count}")

        # Check each module contributed
        for i in range(num_modules):
            module_id = f"MULTI-{i:03d}"
            count = module_counts[module_id]
            self.assertGreater(count, 3,
                               f"Module {module_id} only sent {count} heartbeats (expected >3)")

        print(f"✓ Total heartbeats: {total_count} from {num_modules} modules")
        print(f"✓ Per-module counts: {dict(module_counts)}")

    def test_no_heartbeat_crosstalk(self):
        """
        Test module IDs are correctly preserved (no crosstalk).

        Requirement: Each heartbeat must have correct module_id
        """
        # Start heartbeats
        for module in self.modules:
            module.state = ModuleState.NORMAL
            module.start_heartbeat()

        time.sleep(3.0)

        messages = self.hb_capture.get_messages()

        # Check all module_ids are valid
        valid_ids = {f"MULTI-{i:03d}" for i in range(len(self.modules))}

        for msg in messages:
            self.assertIn(msg.module_id, valid_ids,
                          f"Invalid module_id: {msg.module_id}")

        unique_ids = {msg.module_id for msg in messages}
        self.assertEqual(len(unique_ids), len(self.modules),
                         f"Not all modules represented: {unique_ids}")

        print(f"✓ No crosstalk: all {len(unique_ids)} module IDs valid")


class TestRigorousPerformance(unittest.TestCase):
    """Rigorous performance benchmarks."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def test_module_startup_time(self):
        """
        Test module startup time.

        Requirement: Module must boot and be ready within 1 second
        """
        executor = MultiThreadedExecutor()

        start_time = time.time()
        module = Module(module_id="PERF-001", executor=executor)
        executor.add_node(module)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Wait for module to be ready (in BOOT state)
        timeout = 2.0
        ready = False
        while time.time() - start_time < timeout:
            if module.state == ModuleState.BOOT:
                ready = True
                break
            time.sleep(0.01)

        boot_time = time.time() - start_time

        executor.shutdown()
        module.destroy_node()

        self.assertTrue(ready, "Module did not reach BOOT state")
        self.assertLess(boot_time, 1.0,
                        f"Module startup took {boot_time:.3f}s (requirement: <1.0s)")

        print(f"✓ Module startup time: {boot_time:.3f}s")

    def test_message_throughput(self):
        """
        Test message encoding/decoding throughput.

        Requirement: Must process >1000 messages/second
        """
        from coven_core.common import Heartbeat, hb_encode, hb_decode

        num_messages = 10000
        hb = Heartbeat(module_id="THROUGHPUT-TEST", seq=0)

        # Encode test
        start_time = time.time()
        for i in range(num_messages):
            hb.seq = i
            _ = hb_encode(hb)
        encode_duration = time.time() - start_time
        encode_rate = num_messages / encode_duration

        # Decode test
        encoded = hb_encode(hb)
        msg = String(data=encoded)

        start_time = time.time()
        for _ in range(num_messages):
            _ = hb_decode(msg)
        decode_duration = time.time() - start_time
        decode_rate = num_messages / decode_duration

        self.assertGreater(encode_rate, 1000,
                           f"Encode rate {encode_rate:.0f} msg/s below 1000 msg/s")
        self.assertGreater(decode_rate, 1000,
                           f"Decode rate {decode_rate:.0f} msg/s below 1000 msg/s")

        print(f"✓ Message encoding: {encode_rate:.0f} msg/s")
        print(f"✓ Message decoding: {decode_rate:.0f} msg/s")


if __name__ == '__main__':
    unittest.main(verbosity=2)
