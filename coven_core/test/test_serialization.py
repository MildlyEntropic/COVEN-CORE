"""
test_serialization.py - Unit tests for COVEN generic serialization module

Tests the generic encode/decode contract for all 12 COVEN message types.
This replaces test_common.py which tested 24 individual functions.

Test categories:
1. Round-trip: encode → decode produces identical object
2. Malformed JSON: decode returns None, doesn't crash
3. Missing fields: decode uses sensible defaults
4. Type coercion: strings to int/bool/float work correctly
5. Edge cases: empty strings, unicode, nested data

Author: Alexander Shultis
Date: December 2025
"""

import unittest
from dataclasses import dataclass, field
from typing import Optional, List

from std_msgs.msg import String

# Import the new generic serializer (to be implemented)
from coven_core.serialization import encode, decode

# Import all message dataclasses from common
from coven_core.common import (
    IdentifyReq, IdentifyRep,
    VerifyReq, VerifyRep,
    Heartbeat,
    MissionRequest, Waypoint,
    TaskReq, TaskAck, TaskStart, TaskComplete,
    BidNotice, BidProposal,
    CoverageGoal, CoverageStatus, BatteryConfig, CoverageConfig,
    Sector, CoverageMissionComplete,
    # Dock-centric messages
    RoverRegistration, RoverRegistrationAck,
    SensorData, VelocityCommand,
    RoverStatusMsg, DockCommand,
)


class TestGenericSerializerContract(unittest.TestCase):
    """Test the generic encode/decode contract."""

    def test_encode_returns_string(self):
        """encode() should return a JSON string."""
        req = IdentifyReq(req_id="test-123")
        result = encode(req)
        self.assertIsInstance(result, str)

    def test_decode_accepts_string_or_ros_msg(self):
        """decode() should accept both raw string and ROS String message."""
        req = IdentifyReq(req_id="test-123")
        encoded = encode(req)

        # Raw string
        decoded1 = decode(encoded, IdentifyReq)
        self.assertIsNotNone(decoded1)
        self.assertEqual(decoded1.req_id, "test-123")

        # ROS String message
        ros_msg = String(data=encoded)
        decoded2 = decode(ros_msg, IdentifyReq)
        self.assertIsNotNone(decoded2)
        self.assertEqual(decoded2.req_id, "test-123")

    def test_decode_malformed_json_returns_none(self):
        """decode() should return None for malformed JSON, not crash."""
        result = decode("not valid json{", IdentifyReq)
        self.assertIsNone(result)

    def test_decode_empty_string_returns_none(self):
        """decode() should return None for empty string."""
        result = decode("", IdentifyReq)
        self.assertIsNone(result)


class TestIdentifyMessages(unittest.TestCase):
    """Test IdentifyReq and IdentifyRep serialization."""

    def test_identify_req_roundtrip(self):
        """IdentifyReq: encode → decode produces identical object."""
        original = IdentifyReq(req_id="test-123")
        encoded = encode(original)
        decoded = decode(encoded, IdentifyReq)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, original.req_id)

    def test_identify_rep_roundtrip(self):
        """IdentifyRep: encode → decode produces identical object."""
        original = IdentifyRep(
            req_id="test-123",
            module_id="Hermione_Granger",
            module_type="ReconRover",
            fw="1.2.3"
        )
        encoded = encode(original)
        decoded = decode(encoded, IdentifyRep)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, original.req_id)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertEqual(decoded.module_type, original.module_type)
        self.assertEqual(decoded.fw, original.fw)

    def test_identify_req_missing_fields_uses_defaults(self):
        """IdentifyReq: missing fields should use empty string default."""
        decoded = decode("{}", IdentifyReq)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "")

    def test_identify_rep_missing_fields_uses_defaults(self):
        """IdentifyRep: missing fields should use empty string defaults."""
        decoded = decode('{"req_id": "test"}', IdentifyRep)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "test")
        self.assertEqual(decoded.module_id, "")
        self.assertEqual(decoded.module_type, "")
        self.assertEqual(decoded.fw, "")


class TestVerifyMessages(unittest.TestCase):
    """Test VerifyReq and VerifyRep serialization."""

    def test_verify_req_roundtrip(self):
        """VerifyReq: encode → decode produces identical object."""
        original = VerifyReq(module_id="Circe")
        encoded = encode(original)
        decoded = decode(encoded, VerifyReq)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)

    def test_verify_rep_roundtrip(self):
        """VerifyRep: encode → decode produces identical object."""
        original = VerifyRep(
            module_id="Circe",
            ok=True,
            reason="All checks passed"
        )
        encoded = encode(original)
        decoded = decode(encoded, VerifyRep)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertTrue(decoded.ok)
        self.assertEqual(decoded.reason, original.reason)

    def test_verify_rep_bool_coercion_from_int(self):
        """VerifyRep: integer 0/1 should coerce to bool."""
        decoded = decode('{"module_id": "test", "ok": 1, "reason": ""}', VerifyRep)
        self.assertTrue(decoded.ok)

        decoded = decode('{"module_id": "test", "ok": 0, "reason": ""}', VerifyRep)
        self.assertFalse(decoded.ok)

    def test_verify_rep_missing_ok_defaults_false(self):
        """VerifyRep: missing 'ok' field should default to False."""
        decoded = decode('{"module_id": "test", "reason": ""}', VerifyRep)
        self.assertIsNotNone(decoded)
        self.assertFalse(decoded.ok)


class TestHeartbeatMessages(unittest.TestCase):
    """Test Heartbeat serialization."""

    def test_heartbeat_roundtrip(self):
        """Heartbeat: encode → decode produces identical object."""
        original = Heartbeat(module_id="Baba_Yaga", seq=42)
        encoded = encode(original)
        decoded = decode(encoded, Heartbeat)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertEqual(decoded.seq, original.seq)

    def test_heartbeat_seq_coercion_from_string(self):
        """Heartbeat: string seq should coerce to int."""
        decoded = decode('{"module_id": "test", "seq": "123"}', Heartbeat)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.seq, 123)
        self.assertIsInstance(decoded.seq, int)

    def test_heartbeat_missing_seq_defaults_zero(self):
        """Heartbeat: missing seq should default to 0."""
        decoded = decode('{"module_id": "test"}', Heartbeat)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.seq, 0)


class TestTaskMessages(unittest.TestCase):
    """Test TaskReq, TaskAck, TaskStart, TaskComplete serialization."""

    def test_task_req_roundtrip(self):
        """TaskReq: encode → decode produces identical object."""
        original = TaskReq(module_id="Elphaba", task="explore_zone_a")
        encoded = encode(original)
        decoded = decode(encoded, TaskReq)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertEqual(decoded.task, original.task)

    def test_task_ack_roundtrip(self):
        """TaskAck: encode → decode produces identical object."""
        original = TaskAck(
            module_id="Elphaba",
            accepted=True,
            reason="Ready to proceed"
        )
        encoded = encode(original)
        decoded = decode(encoded, TaskAck)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertTrue(decoded.accepted)
        self.assertEqual(decoded.reason, original.reason)

    def test_task_ack_default_reason_empty(self):
        """TaskAck: default reason should be empty string."""
        original = TaskAck(module_id="test", accepted=True)
        encoded = encode(original)
        decoded = decode(encoded, TaskAck)

        self.assertEqual(decoded.reason, "")

    def test_task_start_roundtrip(self):
        """TaskStart: encode → decode produces identical object."""
        original = TaskStart(module_id="Glinda", task="explore_zone_b")
        encoded = encode(original)
        decoded = decode(encoded, TaskStart)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertEqual(decoded.task, original.task)

    def test_task_complete_roundtrip(self):
        """TaskComplete: encode → decode produces identical object."""
        original = TaskComplete(
            module_id="Glinda",
            task="explore_zone_b",
            success=True,
            note="Exploration complete: 85.3% coverage",
            map_data="base64encodeddata",
            map_yaml="base64encodedyaml",
            exploration_metrics={"coverage": 0.853, "duration": 120.5}
        )
        encoded = encode(original)
        decoded = decode(encoded, TaskComplete)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertEqual(decoded.task, original.task)
        self.assertTrue(decoded.success)
        self.assertEqual(decoded.note, original.note)
        self.assertEqual(decoded.map_data, original.map_data)
        self.assertEqual(decoded.map_yaml, original.map_yaml)
        self.assertEqual(decoded.exploration_metrics["coverage"], 0.853)
        self.assertEqual(decoded.exploration_metrics["duration"], 120.5)

    def test_task_complete_defaults(self):
        """TaskComplete: missing fields should use sensible defaults."""
        decoded = decode('{"module_id": "test", "task": "test_task"}', TaskComplete)

        self.assertIsNotNone(decoded)
        self.assertTrue(decoded.success)  # default True
        self.assertEqual(decoded.note, "")
        self.assertEqual(decoded.map_data, "")
        self.assertEqual(decoded.map_yaml, "")
        self.assertEqual(decoded.exploration_metrics, {})

    def test_task_complete_nested_metrics(self):
        """TaskComplete: complex nested exploration_metrics should survive round-trip."""
        metrics = {
            "coverage": 0.95,
            "duration": 300.5,
            "nested": {
                "data": [1, 2, 3],
                "more": {"deep": "value"}
            }
        }
        original = TaskComplete(
            module_id="test",
            task="complex",
            exploration_metrics=metrics
        )
        encoded = encode(original)
        decoded = decode(encoded, TaskComplete)

        self.assertEqual(decoded.exploration_metrics["coverage"], 0.95)
        self.assertEqual(decoded.exploration_metrics["nested"]["data"], [1, 2, 3])
        self.assertEqual(decoded.exploration_metrics["nested"]["more"]["deep"], "value")


class TestBiddingMessages(unittest.TestCase):
    """Test BidNotice and BidProposal serialization."""

    def test_bid_notice_roundtrip(self):
        """BidNotice: encode → decode produces identical object."""
        original = BidNotice(
            task_id="task_abc123",
            task="explore_sector_a",
            deadline=2.5
        )
        encoded = encode(original)
        decoded = decode(encoded, BidNotice)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.task_id, original.task_id)
        self.assertEqual(decoded.task, original.task)
        self.assertAlmostEqual(decoded.deadline, original.deadline)

    def test_bid_notice_default_deadline(self):
        """BidNotice: missing deadline should default to 2.0."""
        decoded = decode('{"task_id": "t1", "task": "explore"}', BidNotice)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.deadline, 2.0)

    def test_bid_proposal_roundtrip(self):
        """BidProposal: encode → decode produces identical object."""
        original = BidProposal(
            task_id="task_abc123",
            module_id="Wanda_Maximoff",
            cost=35.5,
            can_execute=True,
            reason=""
        )
        encoded = encode(original)
        decoded = decode(encoded, BidProposal)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.task_id, original.task_id)
        self.assertEqual(decoded.module_id, original.module_id)
        self.assertAlmostEqual(decoded.cost, original.cost)
        self.assertTrue(decoded.can_execute)
        self.assertEqual(decoded.reason, "")

    def test_bid_proposal_cannot_execute(self):
        """BidProposal: can_execute=False with reason should survive round-trip."""
        original = BidProposal(
            task_id="task_abc123",
            module_id="Wanda_Maximoff",
            cost=999.0,
            can_execute=False,
            reason="Module in FIELD_OPS state"
        )
        encoded = encode(original)
        decoded = decode(encoded, BidProposal)

        self.assertFalse(decoded.can_execute)
        self.assertEqual(decoded.reason, "Module in FIELD_OPS state")

    def test_bid_proposal_defaults(self):
        """BidProposal: missing fields should use sensible defaults."""
        decoded = decode('{"task_id": "t1", "module_id": "m1"}', BidProposal)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.cost, 999.0)  # high default cost
        self.assertTrue(decoded.can_execute)  # default true (bidding implies capability)
        self.assertEqual(decoded.reason, "")


class TestMissionRequestMessages(unittest.TestCase):
    """Test MissionRequest serialization (complex with nested Waypoints)."""

    def test_mission_req_simple_roundtrip(self):
        """MissionRequest: simple task without waypoints."""
        original = MissionRequest(task="explore")
        encoded = encode(original)
        decoded = decode(encoded, MissionRequest)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.task, "explore")
        self.assertEqual(decoded.waypoints, [])
        self.assertTrue(decoded.return_to_dock)

    def test_mission_req_with_waypoints_roundtrip(self):
        """MissionRequest: with waypoints should survive round-trip."""
        waypoints = [
            Waypoint(type="move", distance=2.0, direction="north"),
            Waypoint(type="turn", angle=90.0),
            Waypoint(type="move", distance=3.0, direction="east"),
        ]
        original = MissionRequest(
            task="explore",
            waypoints=waypoints,
            return_to_dock=False
        )
        encoded = encode(original)
        decoded = decode(encoded, MissionRequest)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.task, "explore")
        self.assertFalse(decoded.return_to_dock)
        self.assertEqual(len(decoded.waypoints), 3)

        # Verify waypoint details
        self.assertEqual(decoded.waypoints[0].type, "move")
        self.assertEqual(decoded.waypoints[0].distance, 2.0)
        self.assertEqual(decoded.waypoints[0].direction, "north")

        self.assertEqual(decoded.waypoints[1].type, "turn")
        self.assertEqual(decoded.waypoints[1].angle, 90.0)

        self.assertEqual(decoded.waypoints[2].type, "move")
        self.assertEqual(decoded.waypoints[2].distance, 3.0)
        self.assertEqual(decoded.waypoints[2].direction, "east")


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error conditions."""

    def test_unicode_handling(self):
        """Unicode characters in strings should survive round-trip."""
        original = IdentifyRep(
            req_id="test-123",
            module_id="RR-\U0001F916",  # Robot emoji
            module_type="ReconRover",
            fw="1.0.0"
        )
        encoded = encode(original)
        decoded = decode(encoded, IdentifyRep)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "RR-\U0001F916")

    def test_empty_json_object(self):
        """Empty JSON object {} should return object with defaults."""
        decoded = decode("{}", IdentifyReq)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "")

        decoded = decode("{}", Heartbeat)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "")
        self.assertEqual(decoded.seq, 0)

    def test_decode_array_returns_none(self):
        """JSON array should return None (we expect objects)."""
        result = decode("[1, 2, 3]", IdentifyReq)
        self.assertIsNone(result)

    def test_decode_null_returns_none(self):
        """JSON null should return None."""
        result = decode("null", IdentifyReq)
        self.assertIsNone(result)

    def test_very_large_numbers(self):
        """Very large numbers should survive round-trip."""
        original = Heartbeat(module_id="test", seq=2**31 - 1)
        encoded = encode(original)
        decoded = decode(encoded, Heartbeat)

        self.assertEqual(decoded.seq, 2**31 - 1)

    def test_negative_numbers(self):
        """Negative numbers should survive round-trip."""
        original = Waypoint(type="turn", angle=-90.0)
        encoded = encode(original)
        decoded = decode(encoded, Waypoint)

        self.assertEqual(decoded.angle, -90.0)


class TestROSMessageCompatibility(unittest.TestCase):
    """Test compatibility with ROS2 String messages."""

    def test_encode_to_ros_msg(self):
        """Encoded string can be wrapped in ROS String message."""
        original = IdentifyReq(req_id="test")
        encoded = encode(original)
        ros_msg = String(data=encoded)

        # Decode from ROS message
        decoded = decode(ros_msg, IdentifyReq)
        self.assertEqual(decoded.req_id, "test")

    def test_decode_handles_ros_msg_data_attribute(self):
        """decode() should handle object with .data attribute."""
        ros_msg = String(data='{"req_id": "from_ros"}')
        decoded = decode(ros_msg, IdentifyReq)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.req_id, "from_ros")


class TestCoverageMessages(unittest.TestCase):
    """Test coverage exploration dataclass serialization."""

    def test_coverage_goal_roundtrip(self):
        """CoverageGoal: encode → decode produces identical object."""
        original = CoverageGoal(
            target_coverage=0.90,
            sector="NE",
            sector_bounds=(0.0, 0.0, 5.0, 5.0),
            max_exploration_time=180.0,
            return_on_low_battery=True,
            battery_return_threshold=0.25
        )
        encoded = encode(original)
        decoded = decode(encoded, CoverageGoal)

        self.assertIsNotNone(decoded)
        self.assertAlmostEqual(decoded.target_coverage, 0.90)
        self.assertEqual(decoded.sector, "NE")
        # JSON converts tuples to lists, so compare as lists
        self.assertEqual(list(decoded.sector_bounds), [0.0, 0.0, 5.0, 5.0])
        self.assertAlmostEqual(decoded.max_exploration_time, 180.0)
        self.assertTrue(decoded.return_on_low_battery)
        self.assertAlmostEqual(decoded.battery_return_threshold, 0.25)

    def test_coverage_goal_defaults(self):
        """CoverageGoal: missing fields should use sensible defaults."""
        decoded = decode('{}', CoverageGoal)

        self.assertIsNotNone(decoded)
        self.assertAlmostEqual(decoded.target_coverage, 0.95)
        self.assertIsNone(decoded.sector)
        self.assertIsNone(decoded.sector_bounds)
        self.assertAlmostEqual(decoded.max_exploration_time, 300.0)
        self.assertTrue(decoded.return_on_low_battery)
        self.assertAlmostEqual(decoded.battery_return_threshold, 0.20)

    def test_coverage_status_roundtrip(self):
        """CoverageStatus: encode → decode produces identical object."""
        original = CoverageStatus(
            module_id="Akko",
            current_coverage=0.45,
            battery_remaining=0.72,
            distance_traveled=15.3,
            frontiers_remaining=8,
            returning_to_dock=False,
            reason="exploring"
        )
        encoded = encode(original)
        decoded = decode(encoded, CoverageStatus)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Akko")
        self.assertAlmostEqual(decoded.current_coverage, 0.45)
        self.assertAlmostEqual(decoded.battery_remaining, 0.72)
        self.assertAlmostEqual(decoded.distance_traveled, 15.3)
        self.assertEqual(decoded.frontiers_remaining, 8)
        self.assertFalse(decoded.returning_to_dock)
        self.assertEqual(decoded.reason, "exploring")

    def test_coverage_status_returning(self):
        """CoverageStatus: returning_to_dock with reason should survive round-trip."""
        original = CoverageStatus(
            module_id="Baba_Yaga",
            current_coverage=0.31,
            battery_remaining=0.18,
            distance_traveled=42.7,
            frontiers_remaining=3,
            returning_to_dock=True,
            reason="low_battery"
        )
        encoded = encode(original)
        decoded = decode(encoded, CoverageStatus)

        self.assertTrue(decoded.returning_to_dock)
        self.assertEqual(decoded.reason, "low_battery")

    def test_battery_config_roundtrip(self):
        """BatteryConfig: encode → decode produces identical object."""
        original = BatteryConfig(
            initial_level=0.8,
            drain_per_meter=0.01,
            drain_per_second_idle=0.0002,
            return_threshold=0.25,
            critical_threshold=0.10,
            recharge_rate=0.15
        )
        encoded = encode(original)
        decoded = decode(encoded, BatteryConfig)

        self.assertIsNotNone(decoded)
        self.assertAlmostEqual(decoded.initial_level, 0.8)
        self.assertAlmostEqual(decoded.drain_per_meter, 0.01)
        self.assertAlmostEqual(decoded.drain_per_second_idle, 0.0002)
        self.assertAlmostEqual(decoded.return_threshold, 0.25)
        self.assertAlmostEqual(decoded.critical_threshold, 0.10)
        self.assertAlmostEqual(decoded.recharge_rate, 0.15)

    def test_sector_roundtrip(self):
        """Sector: encode → decode produces identical object."""
        original = Sector(
            name="NE",
            bounds=(-2.5, 0.0, 2.5, 5.0),
            assigned_to="Hermione_Granger",
            coverage=0.67
        )
        encoded = encode(original)
        decoded = decode(encoded, Sector)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.name, "NE")
        # JSON converts tuples to lists, so compare as lists
        self.assertEqual(list(decoded.bounds), [-2.5, 0.0, 2.5, 5.0])
        self.assertEqual(decoded.assigned_to, "Hermione_Granger")
        self.assertAlmostEqual(decoded.coverage, 0.67)

    def test_sector_unassigned(self):
        """Sector: unassigned sector should have None assigned_to."""
        original = Sector(name="SW", bounds=(0.0, 0.0, 5.0, 5.0))
        encoded = encode(original)
        decoded = decode(encoded, Sector)

        self.assertIsNone(decoded.assigned_to)
        self.assertAlmostEqual(decoded.coverage, 0.0)

    def test_coverage_mission_complete_roundtrip(self):
        """CoverageMissionComplete: encode → decode produces identical object."""
        original = CoverageMissionComplete(
            success=True,
            total_coverage=0.96,
            target_coverage=0.95,
            total_time=542.3,
            rovers_dispatched=2,
            dispatch_cycles=4
        )
        encoded = encode(original)
        decoded = decode(encoded, CoverageMissionComplete)

        self.assertIsNotNone(decoded)
        self.assertTrue(decoded.success)
        self.assertAlmostEqual(decoded.total_coverage, 0.96)
        self.assertAlmostEqual(decoded.target_coverage, 0.95)
        self.assertAlmostEqual(decoded.total_time, 542.3)
        self.assertEqual(decoded.rovers_dispatched, 2)
        self.assertEqual(decoded.dispatch_cycles, 4)

    def test_mission_request_with_coverage_goal(self):
        """MissionRequest: coverage_goal field should survive round-trip."""
        coverage_goal = CoverageGoal(
            target_coverage=0.90,
            sector="NW",
            sector_bounds=(-5.0, 0.0, 0.0, 5.0),
            max_exploration_time=240.0
        )
        original = MissionRequest(
            task="coverage",
            coverage_goal=coverage_goal,
            return_to_dock=True
        )
        encoded = encode(original)
        decoded = decode(encoded, MissionRequest)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.task, "coverage")
        self.assertIsNotNone(decoded.coverage_goal)
        self.assertAlmostEqual(decoded.coverage_goal.target_coverage, 0.90)
        self.assertEqual(decoded.coverage_goal.sector, "NW")
        # JSON converts tuples to lists, so compare as lists
        self.assertEqual(list(decoded.coverage_goal.sector_bounds), [-5.0, 0.0, 0.0, 5.0])


class TestDockCentricMessages(unittest.TestCase):
    """Test dock-centric architecture message serialization."""

    def test_rover_registration_roundtrip(self):
        """RoverRegistration: encode → decode produces identical object."""
        original = RoverRegistration(
            module_id="Circe",
            module_type="lidar_rover",
            firmware_version="2.1.0",
            capabilities=["lidar", "odom", "imu"],
            initial_battery=0.95
        )
        encoded = encode(original)
        decoded = decode(encoded, RoverRegistration)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Circe")
        self.assertEqual(decoded.module_type, "lidar_rover")
        self.assertEqual(decoded.firmware_version, "2.1.0")
        self.assertEqual(decoded.capabilities, ["lidar", "odom", "imu"])
        self.assertAlmostEqual(decoded.initial_battery, 0.95)

    def test_rover_registration_defaults(self):
        """RoverRegistration: missing fields should use sensible defaults."""
        decoded = decode('{"module_id": "TestRover"}', RoverRegistration)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "TestRover")
        self.assertEqual(decoded.module_type, "lidar_rover")
        self.assertEqual(decoded.firmware_version, "1.0.0")
        self.assertEqual(decoded.capabilities, ["lidar", "odom"])
        self.assertAlmostEqual(decoded.initial_battery, 1.0)

    def test_rover_registration_ack_roundtrip(self):
        """RoverRegistrationAck: encode → decode produces identical object."""
        original = RoverRegistrationAck(
            module_id="Circe",
            accepted=True,
            assigned_namespace="Circe",
            reason=""
        )
        encoded = encode(original)
        decoded = decode(encoded, RoverRegistrationAck)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Circe")
        self.assertTrue(decoded.accepted)
        self.assertEqual(decoded.assigned_namespace, "Circe")
        self.assertEqual(decoded.reason, "")

    def test_rover_registration_ack_rejected(self):
        """RoverRegistrationAck: rejected registration with reason."""
        original = RoverRegistrationAck(
            module_id="Unknown",
            accepted=False,
            assigned_namespace="",
            reason="Unknown rover type"
        )
        encoded = encode(original)
        decoded = decode(encoded, RoverRegistrationAck)

        self.assertFalse(decoded.accepted)
        self.assertEqual(decoded.reason, "Unknown rover type")

    def test_sensor_data_roundtrip(self):
        """SensorData: encode → decode produces identical object."""
        original = SensorData(
            module_id="Hecate",
            timestamp=1234567890.123,
            scan_ranges=[1.0, 1.5, 2.0, 2.5, 3.0],
            scan_angle_min=0.0,
            scan_angle_max=3.14159,
            scan_angle_increment=0.0175,
            odom_x=1.5,
            odom_y=2.3,
            odom_theta=0.785,
            odom_vx=0.2,
            odom_vtheta=0.1,
            battery_level=0.82
        )
        encoded = encode(original)
        decoded = decode(encoded, SensorData)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Hecate")
        self.assertAlmostEqual(decoded.timestamp, 1234567890.123)
        self.assertEqual(decoded.scan_ranges, [1.0, 1.5, 2.0, 2.5, 3.0])
        self.assertAlmostEqual(decoded.scan_angle_min, 0.0)
        self.assertAlmostEqual(decoded.scan_angle_max, 3.14159)
        self.assertAlmostEqual(decoded.odom_x, 1.5)
        self.assertAlmostEqual(decoded.odom_y, 2.3)
        self.assertAlmostEqual(decoded.odom_theta, 0.785)
        self.assertAlmostEqual(decoded.odom_vx, 0.2)
        self.assertAlmostEqual(decoded.odom_vtheta, 0.1)
        self.assertAlmostEqual(decoded.battery_level, 0.82)

    def test_sensor_data_defaults(self):
        """SensorData: missing fields should use sensible defaults."""
        decoded = decode('{"module_id": "TestRover"}', SensorData)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "TestRover")
        self.assertAlmostEqual(decoded.timestamp, 0.0)
        self.assertEqual(decoded.scan_ranges, [])
        self.assertAlmostEqual(decoded.odom_x, 0.0)
        self.assertAlmostEqual(decoded.odom_y, 0.0)
        self.assertAlmostEqual(decoded.battery_level, 1.0)

    def test_sensor_data_large_scan(self):
        """SensorData: large scan array (360 points) should survive round-trip."""
        scan_ranges = [float(i % 10) for i in range(360)]
        original = SensorData(
            module_id="Scanner",
            scan_ranges=scan_ranges
        )
        encoded = encode(original)
        decoded = decode(encoded, SensorData)

        self.assertEqual(len(decoded.scan_ranges), 360)
        self.assertEqual(decoded.scan_ranges[0], 0.0)
        self.assertEqual(decoded.scan_ranges[9], 9.0)
        self.assertEqual(decoded.scan_ranges[10], 0.0)

    def test_velocity_command_roundtrip(self):
        """VelocityCommand: encode → decode produces identical object."""
        original = VelocityCommand(
            module_id="Morrigan",
            linear_x=0.3,
            angular_z=-0.5,
            timestamp=1234567890.0,
            timeout=0.25
        )
        encoded = encode(original)
        decoded = decode(encoded, VelocityCommand)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Morrigan")
        self.assertAlmostEqual(decoded.linear_x, 0.3)
        self.assertAlmostEqual(decoded.angular_z, -0.5)
        self.assertAlmostEqual(decoded.timestamp, 1234567890.0)
        self.assertAlmostEqual(decoded.timeout, 0.25)

    def test_velocity_command_defaults(self):
        """VelocityCommand: missing fields should default to stopped."""
        decoded = decode('{"module_id": "Rover1"}', VelocityCommand)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Rover1")
        self.assertAlmostEqual(decoded.linear_x, 0.0)
        self.assertAlmostEqual(decoded.angular_z, 0.0)
        self.assertAlmostEqual(decoded.timeout, 0.5)

    def test_rover_status_roundtrip(self):
        """RoverStatusMsg: encode → decode produces identical object."""
        original = RoverStatusMsg(
            module_id="Lorelei",
            state="ACTIVE",
            battery_level=0.65,
            is_moving=True,
            last_cmd_age=0.05,
            error_msg=""
        )
        encoded = encode(original)
        decoded = decode(encoded, RoverStatusMsg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Lorelei")
        self.assertEqual(decoded.state, "ACTIVE")
        self.assertAlmostEqual(decoded.battery_level, 0.65)
        self.assertTrue(decoded.is_moving)
        self.assertAlmostEqual(decoded.last_cmd_age, 0.05)
        self.assertEqual(decoded.error_msg, "")

    def test_rover_status_error_state(self):
        """RoverStatusMsg: error state with message should survive round-trip."""
        original = RoverStatusMsg(
            module_id="BrokenBot",
            state="ERROR",
            battery_level=0.10,
            is_moving=False,
            last_cmd_age=5.0,
            error_msg="Motor driver fault detected"
        )
        encoded = encode(original)
        decoded = decode(encoded, RoverStatusMsg)

        self.assertEqual(decoded.state, "ERROR")
        self.assertFalse(decoded.is_moving)
        self.assertEqual(decoded.error_msg, "Motor driver fault detected")

    def test_rover_status_defaults(self):
        """RoverStatusMsg: missing fields should use sensible defaults."""
        decoded = decode('{"module_id": "Rover1"}', RoverStatusMsg)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.state, "READY")
        self.assertAlmostEqual(decoded.battery_level, 1.0)
        self.assertFalse(decoded.is_moving)
        self.assertAlmostEqual(decoded.last_cmd_age, 0.0)
        self.assertEqual(decoded.error_msg, "")

    def test_dock_command_roundtrip(self):
        """DockCommand: encode → decode produces identical object."""
        original = DockCommand(
            module_id="Yubaba",
            command="start",
            parameters={"target": "exploration", "sector": "NE"}
        )
        encoded = encode(original)
        decoded = decode(encoded, DockCommand)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.module_id, "Yubaba")
        self.assertEqual(decoded.command, "start")
        self.assertEqual(decoded.parameters["target"], "exploration")
        self.assertEqual(decoded.parameters["sector"], "NE")

    def test_dock_command_simple(self):
        """DockCommand: simple command without parameters."""
        original = DockCommand(
            module_id="Kiki",
            command="stop"
        )
        encoded = encode(original)
        decoded = decode(encoded, DockCommand)

        self.assertEqual(decoded.module_id, "Kiki")
        self.assertEqual(decoded.command, "stop")
        self.assertEqual(decoded.parameters, {})

    def test_dock_command_return(self):
        """DockCommand: return command with coordinates."""
        original = DockCommand(
            module_id="Akko",
            command="return",
            parameters={"dock_x": 0.0, "dock_y": 0.0, "urgent": True}
        )
        encoded = encode(original)
        decoded = decode(encoded, DockCommand)

        self.assertEqual(decoded.command, "return")
        self.assertAlmostEqual(decoded.parameters["dock_x"], 0.0)
        self.assertAlmostEqual(decoded.parameters["dock_y"], 0.0)
        self.assertTrue(decoded.parameters["urgent"])


if __name__ == '__main__':
    unittest.main()
