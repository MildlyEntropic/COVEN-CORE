#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
test_rover_codec.py — Verify the rover-side Python codec produces the same
wire format the Rust firmware emits, by round-tripping each rover→dock
message through (encode → frame parse → decode) and asserting the recovered
fields match what was sent.

If these tests pass, the simulation rover-proxy can speak the COVEN protocol
to the dock and the dock cannot tell it apart from a Rust-firmware rover at
the protocol layer.

Author: Alexander Shultis
Date: April 2026
"""

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coven_core.frame_codec import (  # noqa: E402
    FrameParser,
    decode_data_frame,
    decode_heartbeat,
    decode_identify_reply,
    validate_frame,
)
from coven_core.rover_codec import (  # noqa: E402
    encode_heartbeat,
    encode_identify_reply,
    encode_odom_data_json,
    encode_scan_data_json,
    encode_task_ack_json,
    encode_task_complete_json,
    encode_task_start_json,
    encode_verify_rep,
)


def parse_one_frame(raw: bytes) -> tuple:
    """Feed raw framed bytes through FrameParser and return the single
    validated (msg_type, payload) tuple it produces."""
    parser = FrameParser()
    frames = parser.feed(raw)
    if len(frames) != 1:
        raise AssertionError(f"expected 1 frame, got {len(frames)}: {frames}")
    return frames[0]


class TestIdentifyReplyRoundTrip(unittest.TestCase):
    def test_identify_reply_round_trip(self):
        frame = encode_identify_reply(
            module_id="witch_alpha",
            module_type="ReconRover",
            firmware="1.2.3",
            battery_pct=85.0,
            status="OK",
            capabilities=0x03,
        )
        msg_type, payload = parse_one_frame(frame)
        self.assertEqual(msg_type, 0x02, "IDENTIFY_REPLY type byte")
        decoded = decode_identify_reply(payload)
        self.assertIsNotNone(decoded, "decode_identify_reply returned None")
        self.assertEqual(decoded["module_id"], "witch_alpha")
        self.assertEqual(decoded["capabilities"], 0x03)
        self.assertEqual(decoded["battery_level"], 85)
        self.assertEqual(decoded["status"], "OK")

    def test_identify_reply_clamps_battery(self):
        frame = encode_identify_reply(
            module_id="m",
            module_type="ReconRover",
            firmware="0.1.0",
            battery_pct=150.0,  # over-range
            status="OK",
            capabilities=0x13,
        )
        _, payload = parse_one_frame(frame)
        decoded = decode_identify_reply(payload)
        self.assertEqual(decoded["battery_level"], 100)

    def test_identify_reply_capabilities_pin(self):
        # The four declared rover classes must round-trip their declared
        # capabilities bytes through this codec without alteration.
        for module_type, caps in [
            ("ReconRover", 0x03),     # Mapping (LiDAR)
            ("ReconRover", 0x05),     # Recon (ultrasonic, no LiDAR)
            ("ReconRover", 0x13),     # Spectral
            ("ReconRover", 0x23),     # Drill
            ("CargoRover", 0x41),     # Cargo (extension)
        ]:
            frame = encode_identify_reply(
                module_id="x", module_type=module_type, firmware="0.1.0",
                battery_pct=80.0, status="OK", capabilities=caps,
            )
            _, payload = parse_one_frame(frame)
            decoded = decode_identify_reply(payload)
            self.assertEqual(decoded["capabilities"], caps,
                             f"capabilities round-trip failed for 0x{caps:02x}")


class TestHeartbeatRoundTrip(unittest.TestCase):
    def test_heartbeat_round_trip(self):
        frame = encode_heartbeat(
            module_id="witch_alpha",
            battery_pct=72.5,
            mission_status="ACTIVE",
            x=1.234,
            y=-2.345,
            theta=0.5,
        )
        msg_type, payload = parse_one_frame(frame)
        self.assertEqual(msg_type, 0x20, "MODULE_HEARTBEAT type byte")
        decoded = decode_heartbeat(payload)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded["module_id"], "witch_alpha")
        self.assertEqual(decoded["battery_pct"], 72)  # u8 truncation
        # Position fixed-point: mm precision; reconstruct what the encoder put on wire.
        # x=1.234 → x_mm=int(1234.0)=1234, decoded back to 1234/1000 = 1.234
        self.assertAlmostEqual(decoded["x"], 1.234, places=3)
        self.assertAlmostEqual(decoded["y"], -2.345, places=3)
        self.assertAlmostEqual(decoded["theta"], 0.5, places=2)

    def test_heartbeat_idle_status(self):
        frame = encode_heartbeat("m", 100.0, "IDLE", 0.0, 0.0, 0.0)
        _, payload = parse_one_frame(frame)
        decoded = decode_heartbeat(payload)
        # Mission status byte 0x00 → IDLE in dock decode
        self.assertEqual(decoded.get("mission_status"), "IDLE")


class TestVerifyRepRoundTrip(unittest.TestCase):
    def test_verify_rep_success(self):
        frame = encode_verify_rep(
            module_id="witch_alpha",
            success=True,
            failed_checks=[],
            note="All systems nominal",
        )
        msg_type, payload = parse_one_frame(frame)
        self.assertEqual(msg_type, 0x10, "DATA_FRAME type byte")
        decoded = decode_data_frame(payload)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.get("type"), "VERIFY_REP")
        self.assertTrue(decoded.get("success"))
        self.assertEqual(decoded.get("module_id"), "witch_alpha")

    def test_verify_rep_failure(self):
        frame = encode_verify_rep(
            module_id="witch_alpha",
            success=False,
            failed_checks=["lidar_timeout", "battery_low"],
            note="Two checks failed",
        )
        _, payload = parse_one_frame(frame)
        decoded = decode_data_frame(payload)
        self.assertEqual(decoded.get("type"), "VERIFY_REP")
        self.assertFalse(decoded.get("success"))


class TestScanDataRoundTrip(unittest.TestCase):
    def test_scan_data_round_trip(self):
        frame = encode_scan_data_json(
            module_id="witch_alpha",
            timestamp=1234.567,
            angle_min=-3.14,
            angle_max=3.14,
            angle_increment=0.01745,
            range_min=0.1,
            range_max=12.0,
            ranges_mm=[500, 600, 700, 800],
        )
        msg_type, payload = parse_one_frame(frame)
        self.assertEqual(msg_type, 0x10)
        decoded = decode_data_frame(payload)
        self.assertIsNotNone(decoded)
        # _decode_sensor_json recognizes the "ScanData" key and tags the result.
        self.assertEqual(decoded.get("type"), "SCAN_DATA")
        self.assertIn("ScanData", decoded)
        self.assertEqual(decoded["ScanData"]["module_id"], "witch_alpha")
        self.assertEqual(decoded["ScanData"]["scan"]["ranges_mm"], [500, 600, 700, 800])


class TestOdomDataRoundTrip(unittest.TestCase):
    def test_odom_data_round_trip(self):
        frame = encode_odom_data_json(
            module_id="witch_alpha",
            timestamp=42.0,
            x=1.5, y=2.5, theta=0.7,
            v_linear=0.3, v_angular=0.05,
        )
        _, payload = parse_one_frame(frame)
        decoded = decode_data_frame(payload)
        self.assertEqual(decoded.get("type"), "ODOM_DATA")
        self.assertIn("OdomData", decoded)
        self.assertEqual(decoded["OdomData"]["module_id"], "witch_alpha")
        self.assertEqual(decoded["OdomData"]["odom"]["x"], 1.5)


class TestTaskMessageRoundTrip(unittest.TestCase):
    def test_task_ack(self):
        frame = encode_task_ack_json(
            module_id="witch_alpha", task_id="t-001", success=True,
        )
        _, payload = parse_one_frame(frame)
        decoded = decode_data_frame(payload)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded["type"], "TASK_ACK")
        self.assertEqual(decoded["module_id"], "witch_alpha")
        self.assertEqual(decoded["task_id"], "t-001")
        self.assertTrue(decoded["success"])

    def test_task_start(self):
        frame = encode_task_start_json(
            module_id="witch_alpha", task_id="t-001", timestamp=12345.0,
        )
        _, payload = parse_one_frame(frame)
        decoded = decode_data_frame(payload)
        self.assertEqual(decoded["type"], "TASK_START")
        self.assertEqual(decoded["task_id"], "t-001")
        self.assertEqual(decoded["timestamp"], 12345.0)

    def test_task_complete(self):
        frame = encode_task_complete_json(
            module_id="witch_alpha", task_id="t-001",
            success=True, coverage=0.87, duration=142.5,
        )
        _, payload = parse_one_frame(frame)
        decoded = decode_data_frame(payload)
        self.assertEqual(decoded["type"], "TASK_COMPLETE")
        self.assertEqual(decoded["task_id"], "t-001")
        self.assertAlmostEqual(decoded["coverage"], 0.87)
        self.assertAlmostEqual(decoded["duration"], 142.5)


class TestWireFormatInvariants(unittest.TestCase):
    """The wire format itself: framed bytes always end in 0x00, never have
    interior 0x00 bytes."""

    def test_framed_bytes_end_in_terminator(self):
        frame = encode_identify_reply(
            "x", "ReconRover", "0.1.0", 50.0, "OK", 0x03,
        )
        self.assertEqual(frame[-1], 0x00, "frame must end with 0x00 terminator")

    def test_no_interior_zeros_in_cobs_block(self):
        frame = encode_identify_reply(
            "x", "ReconRover", "0.1.0", 50.0, "OK", 0x03,
        )
        # Layout: [LEN_HI][LEN_LO][COBS data][0x00]
        cobs_block = frame[2:-1]
        self.assertNotIn(0x00, cobs_block, "COBS block must not contain 0x00")

    def test_length_prefix_matches_cobs_block(self):
        frame = encode_heartbeat("x", 50.0, "IDLE", 0.0, 0.0, 0.0)
        declared = (frame[0] << 8) | frame[1]
        actual = len(frame) - 3  # subtract the 2 length bytes and the terminator
        self.assertEqual(declared, actual,
                         f"length prefix {declared} != actual COBS block length {actual}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
