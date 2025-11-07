"""
test_common.py — Unit tests for COVEN common.py module

Tests all encode/decode functions and dataclasses to ensure:
- Proper JSON serialization/deserialization
- Handling of malformed JSON
- Handling of missing fields
- Type conversions (e.g., string to bool, string to int)
- Dataclass initialization

Author: Alexander Shultis
Date: November 2025
"""

import unittest
from std_msgs.msg import String
from coven_core.common import (
    # Dataclasses
    IdentifyReq, IdentifyRep,
    VerifyReq, VerifyRep,
    Heartbeat,
    MissionRequest,
    TaskReq, TaskAck, TaskStart, TaskComplete,
    # Encode functions
    ident_req_encode, ident_rep_encode,
    verify_req_encode, verify_rep_encode,
    hb_encode,
    mission_req_encode,
    task_req_encode, task_ack_encode, task_start_encode, task_complete_encode,
    # Decode functions
    ident_req_decode, ident_rep_decode,
    verify_req_decode, verify_rep_decode,
    hb_decode,
    mission_req_decode,
    task_req_decode, task_ack_decode, task_start_decode, task_complete_decode,
)


class TestIdentifyMessages(unittest.TestCase):
    """Test IdentifyReq and IdentifyRep encode/decode."""

    def test_ident_req_encode_decode(self):
        """Test IdentifyReq round-trip encoding/decoding."""
        req = IdentifyReq(req_id="test-123")
        encoded = ident_req_encode(req)
        msg = String(data=encoded)
        decoded = ident_req_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "test-123")

    def test_ident_rep_encode_decode(self):
        """Test IdentifyRep round-trip encoding/decoding."""
        rep = IdentifyRep(
            req_id="test-123",
            module_id="RR-abc123",
            module_type="ReconRover",
            fw="1.2.3"
        )
        encoded = ident_rep_encode(rep)
        msg = String(data=encoded)
        decoded = ident_rep_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "test-123")
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertEqual(decoded.module_type, "ReconRover")
        self.assertEqual(decoded.fw, "1.2.3")

    def test_ident_req_malformed_json(self):
        """Test IdentifyReq decode with malformed JSON."""
        msg = String(data="not valid json{")
        decoded = ident_req_decode(msg)
        self.assertIsNone(decoded)

    def test_ident_rep_missing_fields(self):
        """Test IdentifyRep decode with missing fields."""
        msg = String(data='{"req_id": "test"}')  # Missing module_id, module_type, fw
        decoded = ident_rep_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "test")
        self.assertEqual(decoded.module_id, "")
        self.assertEqual(decoded.module_type, "")
        self.assertEqual(decoded.fw, "")


class TestVerifyMessages(unittest.TestCase):
    """Test VerifyReq and VerifyRep encode/decode."""

    def test_verify_req_encode_decode(self):
        """Test VerifyReq round-trip encoding/decoding."""
        req = VerifyReq(module_id="RR-abc123")
        encoded = verify_req_encode(req)
        msg = String(data=encoded)
        decoded = verify_req_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")

    def test_verify_rep_encode_decode(self):
        """Test VerifyRep round-trip encoding/decoding."""
        rep = VerifyRep(
            module_id="RR-abc123",
            ok=True,
            reason="All checks passed"
        )
        encoded = verify_rep_encode(rep)
        msg = String(data=encoded)
        decoded = verify_rep_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertTrue(decoded.ok)
        self.assertEqual(decoded.reason, "All checks passed")

    def test_verify_rep_bool_conversion(self):
        """Test VerifyRep decode with various boolean values."""
        # Test with integer 0/1
        msg = String(data='{"module_id": "test", "ok": 1, "reason": ""}')
        decoded = verify_rep_decode(msg)
        self.assertTrue(decoded.ok)

        msg = String(data='{"module_id": "test", "ok": 0, "reason": ""}')
        decoded = verify_rep_decode(msg)
        self.assertFalse(decoded.ok)

    def test_verify_req_malformed_json(self):
        """Test VerifyReq decode with malformed JSON."""
        msg = String(data="}{invalid")
        decoded = verify_req_decode(msg)
        self.assertIsNone(decoded)


class TestHeartbeatMessages(unittest.TestCase):
    """Test Heartbeat encode/decode."""

    def test_heartbeat_encode_decode(self):
        """Test Heartbeat round-trip encoding/decoding."""
        hb = Heartbeat(module_id="RR-abc123", seq=42)
        encoded = hb_encode(hb)
        msg = String(data=encoded)
        decoded = hb_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertEqual(decoded.seq, 42)

    def test_heartbeat_seq_type_conversion(self):
        """Test Heartbeat decode with string sequence number."""
        msg = String(data='{"module_id": "test", "seq": "123"}')
        decoded = hb_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.seq, 123)
        self.assertIsInstance(decoded.seq, int)

    def test_heartbeat_malformed_json(self):
        """Test Heartbeat decode with malformed JSON."""
        msg = String(data="[not an object]")
        decoded = hb_decode(msg)
        self.assertIsNone(decoded)


class TestTaskMessages(unittest.TestCase):
    """Test Task-related message encode/decode."""

    def test_mission_req_encode_decode(self):
        """Test MissionRequest round-trip encoding/decoding."""
        req = MissionRequest(task="explore_sector_a")
        encoded = mission_req_encode(req)
        msg = String(data=encoded)
        decoded = mission_req_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.task, "explore_sector_a")

    def test_task_req_encode_decode(self):
        """Test TaskReq round-trip encoding/decoding."""
        req = TaskReq(module_id="RR-abc123", task="explore_sector_a")
        encoded = task_req_encode(req)
        msg = String(data=encoded)
        decoded = task_req_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertEqual(decoded.task, "explore_sector_a")

    def test_task_ack_encode_decode(self):
        """Test TaskAck round-trip encoding/decoding."""
        ack = TaskAck(
            module_id="RR-abc123",
            accepted=True,
            reason="Ready to proceed"
        )
        encoded = task_ack_encode(ack)
        msg = String(data=encoded)
        decoded = task_ack_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertTrue(decoded.accepted)
        self.assertEqual(decoded.reason, "Ready to proceed")

    def test_task_ack_default_reason(self):
        """Test TaskAck with default reason field."""
        ack = TaskAck(module_id="RR-abc123", accepted=True)
        encoded = task_ack_encode(ack)
        msg = String(data=encoded)
        decoded = task_ack_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.reason, "")

    def test_task_start_encode_decode(self):
        """Test TaskStart round-trip encoding/decoding."""
        ts = TaskStart(module_id="RR-abc123", task="explore_sector_a")
        encoded = task_start_encode(ts)
        msg = String(data=encoded)
        decoded = task_start_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertEqual(decoded.task, "explore_sector_a")

    def test_task_complete_encode_decode(self):
        """Test TaskComplete round-trip encoding/decoding."""
        tc = TaskComplete(
            module_id="RR-abc123",
            task="explore_sector_a",
            success=True,
            note="Exploration complete: 85.3% coverage",
            map_data="base64encodeddata",
            map_yaml="base64encodedyaml",
            exploration_metrics={"coverage": 0.853, "duration": 120.5}
        )
        encoded = task_complete_encode(tc)
        msg = String(data=encoded)
        decoded = task_complete_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-abc123")
        self.assertEqual(decoded.task, "explore_sector_a")
        self.assertTrue(decoded.success)
        self.assertEqual(decoded.note, "Exploration complete: 85.3% coverage")
        self.assertEqual(decoded.map_data, "base64encodeddata")
        self.assertEqual(decoded.map_yaml, "base64encodedyaml")
        self.assertEqual(decoded.exploration_metrics["coverage"], 0.853)
        self.assertEqual(decoded.exploration_metrics["duration"], 120.5)

    def test_task_complete_defaults(self):
        """Test TaskComplete with default values."""
        tc = TaskComplete(module_id="RR-abc123", task="test_task")
        encoded = task_complete_encode(tc)
        msg = String(data=encoded)
        decoded = task_complete_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertTrue(decoded.success)
        self.assertEqual(decoded.note, "")
        self.assertEqual(decoded.map_data, "")
        self.assertEqual(decoded.map_yaml, "")
        self.assertEqual(decoded.exploration_metrics, {})

    def test_task_complete_malformed_json(self):
        """Test TaskComplete decode with malformed JSON."""
        msg = String(data="garbage data")
        decoded = task_complete_decode(msg)
        self.assertIsNone(decoded)


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error conditions."""

    def test_empty_string_decode(self):
        """Test decode functions with empty string."""
        msg = String(data="")

        self.assertIsNone(ident_req_decode(msg))
        self.assertIsNone(verify_req_decode(msg))
        self.assertIsNone(hb_decode(msg))
        self.assertIsNone(task_req_decode(msg))
        self.assertIsNone(task_complete_decode(msg))

    def test_empty_json_object(self):
        """Test decode functions with empty JSON object."""
        msg = String(data="{}")

        # These should not crash, should return objects with default values
        ident_req = ident_req_decode(msg)
        self.assertIsNotNone(ident_req)
        self.assertEqual(ident_req.req_id, "")

        verify_req = verify_req_decode(msg)
        self.assertIsNotNone(verify_req)
        self.assertEqual(verify_req.module_id, "")

        hb = hb_decode(msg)
        self.assertIsNotNone(hb)
        self.assertEqual(hb.module_id, "")
        self.assertEqual(hb.seq, 0)

    def test_unicode_handling(self):
        """Test handling of unicode characters in strings."""
        rep = IdentifyRep(
            req_id="test-123",
            module_id="RR-🤖",
            module_type="ReconRover",
            fw="1.0.0"
        )
        encoded = ident_rep_encode(rep)
        msg = String(data=encoded)
        decoded = ident_rep_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-🤖")

    def test_large_exploration_metrics(self):
        """Test TaskComplete with complex exploration metrics."""
        metrics = {
            "coverage": 0.95,
            "duration": 300.5,
            "distance_traveled": 45.2,
            "frontiers_explored": 12,
            "map_resolution": 0.05,
            "cells_explored": 8500,
            "nested": {
                "data": [1, 2, 3],
                "more": {"deep": "value"}
            }
        }
        tc = TaskComplete(
            module_id="RR-abc123",
            task="complex_exploration",
            exploration_metrics=metrics
        )
        encoded = task_complete_encode(tc)
        msg = String(data=encoded)
        decoded = task_complete_decode(msg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.exploration_metrics["coverage"], 0.95)
        self.assertEqual(decoded.exploration_metrics["nested"]["data"], [1, 2, 3])
        self.assertEqual(decoded.exploration_metrics["nested"]["more"]["deep"], "value")


if __name__ == '__main__':
    unittest.main()
