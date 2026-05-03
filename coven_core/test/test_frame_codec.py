# SPDX-License-Identifier: MIT
"""
test_frame_codec.py — Unit tests for COVEN binary frame codec.

Tests the COBS framing, CRC, frame building/validation, message
encoding/decoding, FrameParser streaming, and BatchChunkAssembler.

Author: Alexander Shultis
Date: March 2026
"""

import struct
import unittest

from coven_core.frame_codec import (
    # Low-level
    _cobs_encode,
    _cobs_decode,
    calculate_crc,
    build_frame,
    validate_frame,
    # Stream parser
    FrameParser,
    # Encoders (dock -> rover)
    encode_identify_request,
    encode_identify_ack,
    encode_verify_ok,
    encode_verify_fail,
    encode_task_request,
    encode_cmd_vel,
    encode_system_ping,
    # Decoders (rover -> dock)
    decode_identify_reply,
    decode_heartbeat,
    decode_data_frame,
    decode_message,
    # Batch assembler
    BatchChunkAssembler,
    # Constants
    MSG_IDENTIFY_REQUEST,
    MSG_IDENTIFY_REPLY,
    MSG_VERIFY_OK,
    MSG_VERIFY_FAIL,
    MSG_IDENTIFY_ACK,
    MSG_DATA_FRAME,
    MSG_MODULE_HEARTBEAT,
    MSG_FAULT_ALERT,
    MSG_SYSTEM_PING,
    SUBTYPE_TASK_MESSAGE,
    SUBTYPE_CMD_VEL,
    SUBTYPE_SENSOR_BATCH_CHUNK,
    CHUNK_TYPE_HEADER,
    CHUNK_TYPE_DATA,
    MAX_PAYLOAD_SIZE,
    capabilities_to_list,
    CAP_ENCODERS,
    CAP_LIDAR,
    CAP_DRILL,
)


class TestCOBSEncoding(unittest.TestCase):
    """Test COBS encode/decode round-trips."""

    def test_empty_data(self):
        """Empty input should round-trip."""
        encoded = _cobs_encode(b'')
        decoded = _cobs_decode(encoded)
        self.assertEqual(decoded, b'')

    def test_no_zeros(self):
        """Data with no zero bytes should round-trip."""
        data = b'\x01\x02\x03\x04\x05'
        encoded = _cobs_encode(data)
        decoded = _cobs_decode(encoded)
        self.assertEqual(decoded, data)
        # COBS output must not contain any 0x00 bytes
        self.assertNotIn(0, encoded)

    def test_all_zeros(self):
        """Data with all zero bytes should round-trip."""
        data = b'\x00\x00\x00'
        encoded = _cobs_encode(data)
        decoded = _cobs_decode(encoded)
        self.assertEqual(decoded, data)
        self.assertNotIn(0, encoded)

    def test_mixed_data(self):
        """Mixed data with zeros interspersed should round-trip."""
        data = b'\x01\x00\x02\x03\x00\x04'
        encoded = _cobs_encode(data)
        decoded = _cobs_decode(encoded)
        self.assertEqual(decoded, data)
        self.assertNotIn(0, encoded)

    def test_single_zero(self):
        """Single zero byte should round-trip."""
        data = b'\x00'
        encoded = _cobs_encode(data)
        decoded = _cobs_decode(encoded)
        self.assertEqual(decoded, data)

    def test_long_non_zero_block(self):
        """Block of 254+ non-zero bytes should round-trip (tests 0xFF code path)."""
        data = bytes(range(1, 256))  # 255 non-zero bytes
        encoded = _cobs_encode(data)
        decoded = _cobs_decode(encoded)
        self.assertEqual(decoded, data)
        self.assertNotIn(0, encoded)

    def test_cobs_decode_rejects_embedded_zero(self):
        """COBS decode should raise on unexpected zero in data."""
        with self.assertRaises(ValueError):
            _cobs_decode(b'\x03\x01\x00\x02')

    def test_cobs_decode_rejects_truncated(self):
        """COBS decode should raise on truncated data."""
        with self.assertRaises(ValueError):
            _cobs_decode(b'\x05\x01\x02')  # code=5 but only 2 data bytes


class TestCRC(unittest.TestCase):
    """Test CRC-8 XOR calculation."""

    def test_crc_empty_payload(self):
        """CRC of empty payload is just the message type."""
        self.assertEqual(calculate_crc(0x42, b''), 0x42)

    def test_crc_known_value(self):
        """CRC of type XOR'd with each payload byte."""
        # 0x01 ^ 0x02 ^ 0x03 = 0x00
        self.assertEqual(calculate_crc(0x01, b'\x02\x03'), 0x00)

    def test_crc_all_zeros(self):
        """CRC with all-zero payload equals the type byte."""
        self.assertEqual(calculate_crc(0xFF, b'\x00\x00\x00'), 0xFF)


class TestFrameBuildAndValidate(unittest.TestCase):
    """Test build_frame and validate_frame round-trip."""

    def test_roundtrip_simple(self):
        """Build frame then validate should return original type and payload."""
        msg_type = 0x20
        payload = b'hello'
        frame = build_frame(msg_type, payload)

        # Frame starts with 2-byte length and ends with 0x00
        self.assertEqual(frame[-1], 0x00)

        # Extract COBS data (between length header and terminator)
        length = struct.unpack('>H', frame[:2])[0]
        cobs_data = frame[2:2 + length]

        decoded_type, decoded_payload = validate_frame(cobs_data)
        self.assertEqual(decoded_type, msg_type)
        self.assertEqual(decoded_payload, payload)

    def test_roundtrip_empty_payload(self):
        """Empty payload should round-trip through build/validate."""
        frame = build_frame(0xFF, b'')
        length = struct.unpack('>H', frame[:2])[0]
        cobs_data = frame[2:2 + length]
        msg_type, payload = validate_frame(cobs_data)
        self.assertEqual(msg_type, 0xFF)
        self.assertEqual(payload, b'')

    def test_payload_too_large_raises(self):
        """Payload exceeding MAX_PAYLOAD_SIZE should raise ValueError."""
        with self.assertRaises(ValueError):
            build_frame(0x01, b'\x00' * (MAX_PAYLOAD_SIZE + 1))

    def test_crc_mismatch_detected(self):
        """Corrupted frame should raise ValueError on CRC mismatch."""
        frame = build_frame(0x10, b'\x01\x02\x03')
        length = struct.unpack('>H', frame[:2])[0]
        cobs_data = bytearray(frame[2:2 + length])

        # Corrupt a byte in the COBS data
        cobs_data[1] ^= 0xFF

        with self.assertRaises(ValueError):
            validate_frame(bytes(cobs_data))


class TestFrameParser(unittest.TestCase):
    """Test streaming FrameParser."""

    def test_single_frame(self):
        """Parser should extract a single complete frame."""
        frame = build_frame(MSG_SYSTEM_PING, b'')
        parser = FrameParser()
        results = parser.feed(frame)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0][0], MSG_SYSTEM_PING)
        self.assertEqual(results[0][1], b'')
        self.assertEqual(parser.frames_received, 1)

    def test_multiple_frames(self):
        """Parser should extract multiple concatenated frames."""
        frame1 = build_frame(MSG_SYSTEM_PING, b'')
        frame2 = build_frame(0x20, b'\x01\x02')
        parser = FrameParser()
        results = parser.feed(frame1 + frame2)
        self.assertEqual(len(results), 2)
        self.assertEqual(parser.frames_received, 2)

    def test_byte_at_a_time(self):
        """Parser should work when fed one byte at a time."""
        frame = build_frame(0x10, b'test')
        parser = FrameParser()
        all_results = []
        for byte in frame:
            results = parser.feed(bytes([byte]))
            all_results.extend(results)
        self.assertEqual(len(all_results), 1)
        self.assertEqual(all_results[0][1], b'test')

    def test_invalid_length_resyncs(self):
        """Parser should resync after invalid (zero) length."""
        parser = FrameParser()
        # Feed zero-length (invalid) then a valid frame
        invalid = b'\x00\x00'
        valid = build_frame(MSG_SYSTEM_PING, b'')
        results = parser.feed(invalid + valid)
        # Should recover and parse the valid frame
        self.assertEqual(len(results), 1)

    def test_corrupted_frame_increments_errors(self):
        """Parser should count CRC errors for corrupted frames."""
        frame = bytearray(build_frame(0x10, b'\x01\x02\x03'))
        # Corrupt the COBS data (byte after length header)
        frame[3] ^= 0xFF
        parser = FrameParser()
        results = parser.feed(bytes(frame))
        self.assertEqual(len(results), 0)
        self.assertEqual(parser.crc_errors, 1)

    def test_reset(self):
        """Reset should clear parser state."""
        parser = FrameParser()
        # Feed partial frame
        parser.feed(b'\x00\x05\x01')
        parser.reset()
        # Parser should be back in WAIT_LEN_HI state
        frame = build_frame(MSG_SYSTEM_PING, b'')
        results = parser.feed(frame)
        self.assertEqual(len(results), 1)


class TestDockToRoverEncoders(unittest.TestCase):
    """Test dock-to-rover message encoders produce parseable frames."""

    def _parse_frame(self, frame_bytes):
        """Helper to parse a complete frame and return (type, payload)."""
        parser = FrameParser()
        results = parser.feed(frame_bytes)
        self.assertEqual(len(results), 1, "Expected exactly one frame")
        return results[0]

    def test_identify_request(self):
        """IDENTIFY_REQUEST should produce valid frame with correct type."""
        frame = encode_identify_request("dock-001", "The_Graeae", "Morrigan")
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_IDENTIFY_REQUEST)
        # Payload should contain the three length-prefixed strings
        self.assertGreater(len(payload), 0)

    def test_identify_ack(self):
        """IDENTIFY_ACK should produce valid frame."""
        frame = encode_identify_ack("dock-001", "Morrigan", "Welcome aboard")
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_IDENTIFY_ACK)

    def test_verify_ok(self):
        """VERIFY_OK should produce valid frame."""
        frame = encode_verify_ok("dock-001", "Morrigan")
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_VERIFY_OK)

    def test_verify_fail(self):
        """VERIFY_FAIL should produce valid frame."""
        frame = encode_verify_fail("dock-001", "Unknown")
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_VERIFY_FAIL)

    def test_task_request(self):
        """TASK_REQUEST should produce valid DATA_FRAME with TASK_MESSAGE subtype."""
        task = {"mission_id": "m-001", "target_x": 2.0, "target_y": 3.0}
        frame = encode_task_request(task)
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_DATA_FRAME)
        self.assertEqual(payload[0], SUBTYPE_TASK_MESSAGE)

    def test_cmd_vel(self):
        """CMD_VEL should produce valid DATA_FRAME with CMD_VEL subtype."""
        frame = encode_cmd_vel(0.5, -0.3)
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_DATA_FRAME)
        self.assertEqual(payload[0], SUBTYPE_CMD_VEL)
        # Verify encoded values
        linear, angular = struct.unpack_from('<ff', payload, 1)
        self.assertAlmostEqual(linear, 0.5, places=5)
        self.assertAlmostEqual(angular, -0.3, places=5)

    def test_system_ping(self):
        """SYSTEM_PING should produce valid frame with empty payload."""
        frame = encode_system_ping()
        msg_type, payload = self._parse_frame(frame)
        self.assertEqual(msg_type, MSG_SYSTEM_PING)
        self.assertEqual(payload, b'')


class TestRoverToDockDecoders(unittest.TestCase):
    """Test rover-to-dock message decoders."""

    def test_decode_identify_reply_valid(self):
        """Valid IDENTIFY_REPLY payload should decode correctly."""
        payload = bytearray()
        payload.extend(b'COV')           # magic
        payload.append(0x01)             # type_byte = ReconRover
        payload.append(0x02)             # rev_byte = 2
        module_id = b'Morrigan'
        payload.append(len(module_id))
        payload.extend(module_id)
        payload.append(85)               # battery 85%
        status = b'OK'
        payload.append(len(status))
        payload.extend(status)
        payload.append(CAP_ENCODERS | CAP_LIDAR)  # capabilities

        result = decode_identify_reply(bytes(payload))
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "IDENTIFY_REPLY")
        self.assertEqual(result["module_id"], "Morrigan")
        self.assertEqual(result["module_type"], "ReconRover")
        self.assertEqual(result["firmware"], "2.0.0")
        self.assertEqual(result["battery_level"], 85.0)
        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["capabilities"], CAP_ENCODERS | CAP_LIDAR)

    def test_decode_identify_reply_bad_magic(self):
        """IDENTIFY_REPLY with wrong magic should return None."""
        payload = b'BAD\x01\x01\x03foo\x64\x02OK'
        self.assertIsNone(decode_identify_reply(payload))

    def test_decode_identify_reply_too_short(self):
        """IDENTIFY_REPLY that's too short should return None."""
        self.assertIsNone(decode_identify_reply(b'COV'))

    def test_decode_heartbeat_valid(self):
        """Valid HEARTBEAT payload should decode correctly."""
        payload = bytearray()
        module_id = b'Louhi'
        payload.append(len(module_id))
        payload.extend(module_id)
        payload.append(72)               # battery 72%
        payload.append(0x01)             # status = ACTIVE
        payload.extend(struct.pack('<i', 1500))   # x_mm = 1500
        payload.extend(struct.pack('<i', -2000))  # y_mm = -2000
        payload.extend(struct.pack('<h', 785))    # theta_mrad = 0.785

        result = decode_heartbeat(bytes(payload))
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "HEARTBEAT")
        self.assertEqual(result["module_id"], "Louhi")
        self.assertEqual(result["battery_pct"], 72.0)
        self.assertEqual(result["mission_status"], "ACTIVE")
        self.assertAlmostEqual(result["x"], 1.5)
        self.assertAlmostEqual(result["y"], -2.0)
        self.assertAlmostEqual(result["theta"], 0.785)

    def test_decode_heartbeat_too_short(self):
        """HEARTBEAT that's too short for position data should return None."""
        self.assertIsNone(decode_heartbeat(b''))
        self.assertIsNone(decode_heartbeat(b'\x03foo'))  # missing position

    def test_decode_data_frame_task_ack(self):
        """DATA_FRAME with TASK_MESSAGE containing TaskAck should decode."""
        import json
        inner = json.dumps({"TaskAck": {"mission_id": "m-001", "accepted": True}})
        payload = bytes([SUBTYPE_TASK_MESSAGE]) + inner.encode('utf-8')
        result = decode_data_frame(payload)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "TASK_ACK")

    def test_decode_data_frame_task_complete(self):
        """DATA_FRAME with TASK_MESSAGE containing TaskComplete should decode."""
        import json
        inner = json.dumps({"TaskComplete": {"mission_id": "m-001", "success": True}})
        payload = bytes([SUBTYPE_TASK_MESSAGE]) + inner.encode('utf-8')
        result = decode_data_frame(payload)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "TASK_COMPLETE")

    def test_decode_data_frame_unknown_subtype(self):
        """DATA_FRAME with unknown subtype should return None."""
        result = decode_data_frame(bytes([0xFE, 0x01, 0x02]))
        self.assertIsNone(result)

    def test_decode_data_frame_empty(self):
        """DATA_FRAME with empty payload should return None."""
        self.assertIsNone(decode_data_frame(b''))


class TestDecodeMessage(unittest.TestCase):
    """Test top-level decode_message dispatcher."""

    def test_system_ping(self):
        """SYSTEM_PING should decode to type dict."""
        result = decode_message(MSG_SYSTEM_PING, b'')
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "SYSTEM_PING")

    def test_fault_alert(self):
        """FAULT_ALERT should decode with hex payload."""
        result = decode_message(MSG_FAULT_ALERT, b'\xDE\xAD')
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "FAULT_ALERT")
        self.assertEqual(result["payload"], "dead")

    def test_unknown_type(self):
        """Unknown message type should return None."""
        result = decode_message(0xAA, b'\x01\x02')
        self.assertIsNone(result)


class TestCapabilities(unittest.TestCase):
    """Test capabilities bitmask to list conversion."""

    def test_single_capability(self):
        self.assertEqual(capabilities_to_list(CAP_LIDAR), ["lidar"])

    def test_multiple_capabilities(self):
        caps = capabilities_to_list(CAP_ENCODERS | CAP_LIDAR | CAP_DRILL)
        self.assertIn("encoders", caps)
        self.assertIn("lidar", caps)
        self.assertIn("drill", caps)
        self.assertEqual(len(caps), 3)

    def test_no_capabilities(self):
        self.assertEqual(capabilities_to_list(0), [])


class TestBatchChunkAssembler(unittest.TestCase):
    """Test binary batch chunk reassembly."""

    def _build_header_chunk(
        self, batch_id=1, total_samples=2, total_chunks=1,
        module_id="Morrigan", mission_id="m-001",
    ):
        """Build a minimal BATCH_HEADER chunk payload."""
        data = bytearray()
        data.extend(struct.pack('<I', batch_id))
        # total_samples is u32 (4 bytes) per the Rust firmware encoder; the
        # decoder in coven_core/frame_codec.py reads '<I'. Earlier versions
        # of this helper used '<H' which masked a width mismatch — fixed.
        data.extend(struct.pack('<I', total_samples))
        data.extend(struct.pack('<H', total_chunks))
        mid = module_id.encode('utf-8')
        data.append(len(mid))
        data.extend(mid)
        msn = mission_id.encode('utf-8')
        data.append(len(msn))
        data.extend(msn)
        # metadata: mission_start(8) + wheel_radius_mm(2) + wheel_base_mm(2) + ticks_per_rev(2)
        data.extend(struct.pack('<d', 1000.0))  # mission_start
        data.extend(struct.pack('<H', 80))       # wheel_radius_mm
        data.extend(struct.pack('<H', 298))      # wheel_base_mm
        data.extend(struct.pack('<H', 816))      # ticks_per_rev
        # sensor: type(1) + config_len(2) + config
        data.append(0x01)  # SENSOR_TYPE_LIDAR
        data.extend(struct.pack('<H', 0))  # no sensor config
        return bytes([CHUNK_TYPE_HEADER]) + bytes(data)

    def _build_data_chunk(self, batch_id=1, chunk_seq=0, samples=None):
        """Build a BATCH_DATA chunk payload with given samples."""
        if samples is None:
            samples = [
                {"timestamp": 1.0, "left_ticks": 10, "right_ticks": 12, "sensor_data": b'\x01\x00'},
                {"timestamp": 2.0, "left_ticks": 20, "right_ticks": 22, "sensor_data": b'\x02\x00'},
            ]
        data = bytearray()
        data.extend(struct.pack('<I', batch_id))
        data.extend(struct.pack('<H', chunk_seq))
        data.extend(struct.pack('<H', len(samples)))
        for s in samples:
            data.extend(struct.pack('<d', s["timestamp"]))
            data.extend(struct.pack('<i', s["left_ticks"]))
            data.extend(struct.pack('<i', s["right_ticks"]))
            sd = s["sensor_data"]
            data.extend(struct.pack('<H', len(sd)))
            data.extend(sd)
        return bytes([CHUNK_TYPE_DATA]) + bytes(data)

    def test_single_chunk_batch(self):
        """Header + 1 data chunk should produce complete batch."""
        assembler = BatchChunkAssembler()
        header = self._build_header_chunk(batch_id=42, total_chunks=1)
        result = assembler.feed_chunk(header)
        self.assertIsNone(result)  # header alone doesn't complete

        data = self._build_data_chunk(batch_id=42, chunk_seq=0)
        result = assembler.feed_chunk(data)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "DATA_BATCH")
        self.assertEqual(result["module_id"], "Morrigan")
        self.assertEqual(result["mission_id"], "m-001")
        self.assertEqual(len(result["batch"]["samples"]), 2)

    def test_multi_chunk_batch(self):
        """Header + 2 data chunks should produce complete batch."""
        assembler = BatchChunkAssembler()
        header = self._build_header_chunk(batch_id=1, total_samples=4, total_chunks=2)
        assembler.feed_chunk(header)

        chunk0 = self._build_data_chunk(batch_id=1, chunk_seq=0)
        result = assembler.feed_chunk(chunk0)
        self.assertIsNone(result)  # still waiting for chunk 1

        chunk1 = self._build_data_chunk(batch_id=1, chunk_seq=1)
        result = assembler.feed_chunk(chunk1)
        self.assertIsNotNone(result)
        self.assertEqual(len(result["batch"]["samples"]), 4)

    def test_data_without_header_dropped(self):
        """Data chunk without header should return None."""
        assembler = BatchChunkAssembler()
        data = self._build_data_chunk(batch_id=1, chunk_seq=0)
        result = assembler.feed_chunk(data)
        self.assertIsNone(result)

    def test_mismatched_batch_id_dropped(self):
        """Data chunk with wrong batch_id should be dropped."""
        assembler = BatchChunkAssembler()
        header = self._build_header_chunk(batch_id=1, total_chunks=1)
        assembler.feed_chunk(header)

        data = self._build_data_chunk(batch_id=999, chunk_seq=0)
        result = assembler.feed_chunk(data)
        self.assertIsNone(result)

    def test_reset_clears_state(self):
        """Reset should allow starting a new batch."""
        assembler = BatchChunkAssembler()
        header = self._build_header_chunk(batch_id=1, total_chunks=1)
        assembler.feed_chunk(header)
        assembler.reset()

        # Data from old batch should be dropped (no header)
        data = self._build_data_chunk(batch_id=1, chunk_seq=0)
        result = assembler.feed_chunk(data)
        self.assertIsNone(result)

    def test_empty_chunk_returns_none(self):
        """Empty chunk data should return None."""
        assembler = BatchChunkAssembler()
        self.assertIsNone(assembler.feed_chunk(b''))


class TestEndToEndFramePipeline(unittest.TestCase):
    """Test full encode -> frame -> parse -> decode pipeline."""

    def test_identify_request_pipeline(self):
        """IDENTIFY_REQUEST: encode -> FrameParser -> decode_message."""
        frame = encode_identify_request("dock-001", "The_Graeae", "Morrigan")
        parser = FrameParser()
        results = parser.feed(frame)
        self.assertEqual(len(results), 1)
        msg_type, payload = results[0]
        self.assertEqual(msg_type, MSG_IDENTIFY_REQUEST)

    def test_cmd_vel_pipeline(self):
        """CMD_VEL: encode -> FrameParser -> decode -> verify values."""
        frame = encode_cmd_vel(1.0, -0.5)
        parser = FrameParser()
        results = parser.feed(frame)
        msg_type, payload = results[0]
        result = decode_message(msg_type, payload)
        # CMD_VEL is a DATA_FRAME, decodes to subtype dispatch
        # The subtype is CMD_VEL which isn't handled by decode_data_frame
        # (only VERIFY_REP, TASK_MESSAGE, SENSOR_DATA, SENSOR_BATCH_CHUNK)
        # so verify at the raw level
        self.assertEqual(msg_type, MSG_DATA_FRAME)
        self.assertEqual(payload[0], SUBTYPE_CMD_VEL)

    def test_multiple_messages_in_stream(self):
        """Multiple different message types in one stream should all parse."""
        frames = (
            encode_system_ping()
            + encode_verify_ok("dock", "rover")
            + encode_cmd_vel(0.0, 0.0)
            + encode_system_ping()
        )
        parser = FrameParser()
        results = parser.feed(frames)
        self.assertEqual(len(results), 4)
        self.assertEqual(results[0][0], MSG_SYSTEM_PING)
        self.assertEqual(results[1][0], MSG_VERIFY_OK)
        self.assertEqual(results[2][0], MSG_DATA_FRAME)
        self.assertEqual(results[3][0], MSG_SYSTEM_PING)
        self.assertEqual(parser.frames_received, 4)
        self.assertEqual(parser.crc_errors, 0)


if __name__ == '__main__':
    unittest.main()
