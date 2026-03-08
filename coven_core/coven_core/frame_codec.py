# SPDX-License-Identifier: MIT
"""
frame_codec.py — COVEN binary frame codec for dock-rover communication.

Mirrors the Rust implementation in rover/src/dock_uart.rs exactly.
The dock Python side uses this to build/parse the same binary frames
that the Rust rover firmware sends/receives over UART.

Frame format (per Interface Specification v0.3 — COBS):
  [LEN_HI] [LEN_LO] [COBS-encoded data] [0x00]
  - LEN:   16-bit big-endian length of COBS-encoded data
  - DATA:  COBS encoding of [TYPE][PAYLOAD][CRC]
  - 0x00:  COBS frame terminator (guaranteed absent in encoded data)
  - CRC:   XOR of TYPE and all PAYLOAD bytes (pre-COBS)

COBS (Consistent Overhead Byte Stuffing) guarantees no 0x00 bytes
appear in the encoded data, making 0x00 an unambiguous delimiter.

Author: Alexander Shultis
Date: March 2026
"""

import json
import logging
import struct
import time
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


# --- Frame constants ---

MAX_PAYLOAD_SIZE = 8192
MAX_FRAME_SIZE = MAX_PAYLOAD_SIZE + 256  # payload + COBS overhead + header


# --- Message types from Interface Specification v0.2 ---

MSG_IDENTIFY_REQUEST = 0x01   # dock -> rover
MSG_IDENTIFY_REPLY = 0x02     # rover -> dock
MSG_VERIFY_OK = 0x03          # dock -> rover
MSG_VERIFY_FAIL = 0x04        # dock -> rover
MSG_IDENTIFY_ACK = 0x05       # dock -> rover
MSG_DATA_FRAME = 0x10         # bidirectional
MSG_MODULE_HEARTBEAT = 0x20   # rover -> dock
MSG_FAULT_ALERT = 0x30        # rover -> dock
MSG_SYSTEM_PING = 0xFF        # bidirectional

# Data frame subtypes (first byte of DATA_FRAME payload)
SUBTYPE_VERIFY_REP = 0x01
SUBTYPE_TASK_MESSAGE = 0x10
SUBTYPE_SENSOR_DATA = 0x20
SUBTYPE_SENSOR_BATCH_CHUNK = 0x21
SUBTYPE_CMD_VEL = 0x30
SUBTYPE_ENABLE_POWER = 0x40

# Batch chunk types (second byte of SENSOR_BATCH_CHUNK payload)
CHUNK_TYPE_HEADER = 0x01
CHUNK_TYPE_DATA = 0x02

# Sensor type tags (matches protocol.rs)
SENSOR_TYPE_NONE = 0x00
SENSOR_TYPE_LIDAR = 0x01
SENSOR_TYPE_CAMERA = 0x02
SENSOR_TYPE_SPECTROMETER = 0x03
SENSOR_TYPE_DRILL = 0x04

# Module type bytes (matches dock_uart.rs encode_message)
MODULE_TYPE_MAP = {
    0x01: "ReconRover",
    0x02: "CargoRover",
    0x03: "DrillRover",
    0x04: "ArduinoBot",
    0x00: "Unknown",
}

# Capability flags (bitmask — reported in IDENTIFY_REPLY)
CAP_ENCODERS = 0x01      # Wheel encoders (odometry)
CAP_LIDAR = 0x02         # 2D LiDAR scanner
CAP_ULTRASONIC = 0x04    # Ultrasonic range sensor
CAP_CAMERA = 0x08        # Camera
CAP_SPECTROMETER = 0x10  # Spectrometer
CAP_DRILL = 0x20         # Drill/sampler

# Default capabilities inferred from module type (backward compat)
_DEFAULT_CAPABILITIES = {
    "ReconRover": CAP_ENCODERS | CAP_LIDAR,
    "CargoRover": CAP_ENCODERS,
    "DrillRover": CAP_ENCODERS | CAP_DRILL,
    "ArduinoBot": CAP_ENCODERS | CAP_ULTRASONIC,
    "Unknown": CAP_ENCODERS,
}

# Mission status bytes (matches dock_uart.rs encode_message)
MISSION_STATUS_MAP = {
    0x00: "IDLE",
    0x01: "ACTIVE",
    0x02: "STARTUP",
    0x03: "RETURNING",
    0xFF: "UNKNOWN",
}


_CAP_NAMES = {
    CAP_ENCODERS: "encoders",
    CAP_LIDAR: "lidar",
    CAP_ULTRASONIC: "ultrasonic",
    CAP_CAMERA: "camera",
    CAP_SPECTROMETER: "spectrometer",
    CAP_DRILL: "drill",
}


def capabilities_to_list(bitmask: int) -> List[str]:
    """Convert a capabilities bitmask to a human-readable list of names."""
    return [name for flag, name in _CAP_NAMES.items() if bitmask & flag]


# ---------------------------------------------------------------------------
# COBS encode / decode (inline — no external dependency)
# ---------------------------------------------------------------------------

def _cobs_encode(data: bytes) -> bytes:
    """COBS encode: guarantees no 0x00 bytes in output."""
    output = bytearray()
    idx = 0

    while idx < len(data):
        # Find next zero byte (or end of data)
        next_zero = data.find(b'\x00', idx)
        if next_zero == -1:
            next_zero = len(data)

        block_len = next_zero - idx

        # Handle blocks > 253 bytes (need to split with 0xFF code)
        while block_len > 253:
            output.append(0xFF)
            output.extend(data[idx:idx + 254])
            idx += 254
            block_len -= 254

        # Write code byte + block
        output.append(block_len + 1)
        output.extend(data[idx:idx + block_len])
        idx += block_len

        # Skip past the zero byte (if one exists)
        if idx < len(data) and data[idx] == 0:
            idx += 1
        else:
            break

    return bytes(output)


def _cobs_decode(data: bytes) -> bytes:
    """COBS decode: inverse of _cobs_encode."""
    output = bytearray()
    idx = 0

    while idx < len(data):
        code = data[idx]
        idx += 1

        if code == 0:
            raise ValueError("Unexpected zero in COBS data")

        # Read code-1 data bytes
        end = idx + code - 1
        if end > len(data):
            raise ValueError("COBS data truncated")
        output.extend(data[idx:end])
        idx = end

        # If code < 0xFF and more data follows, insert implicit zero
        if code < 0xFF and idx < len(data):
            output.append(0)

    return bytes(output)


# ---------------------------------------------------------------------------
# Low-level frame operations
# ---------------------------------------------------------------------------

def calculate_crc(msg_type: int, payload: bytes) -> int:
    """Calculate CRC-8 (XOR of type and all payload bytes)."""
    crc = msg_type
    for b in payload:
        crc ^= b
    return crc & 0xFF


def build_frame(msg_type: int, payload: bytes) -> bytes:
    """Build a complete COBS-framed binary frame from message type and payload.

    Wire format: [LEN_HI][LEN_LO][COBS data][0x00]
    """
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"Payload too large: {len(payload)} bytes (max {MAX_PAYLOAD_SIZE})"
        )

    # Build raw block: [TYPE] [PAYLOAD] [CRC]
    crc = calculate_crc(msg_type, payload)
    raw = bytes([msg_type]) + payload + bytes([crc])

    # COBS encode (no zeros in output)
    encoded = _cobs_encode(raw)

    # Length prefix (big-endian u16) + encoded + 0x00 terminator
    length = len(encoded)
    return struct.pack('>H', length) + encoded + b'\x00'


def validate_frame(cobs_data: bytes) -> tuple:
    """Validate and extract (msg_type, payload) from COBS-encoded data.

    Input is the COBS-encoded block (without length prefix or terminator).
    Returns (msg_type, payload) on success, raises ValueError on failure.
    """
    # COBS decode
    raw = _cobs_decode(cobs_data)

    if len(raw) < 2:
        raise ValueError(f"Decoded frame too short: {len(raw)} bytes")

    msg_type = raw[0]
    payload = raw[1:-1]
    received_crc = raw[-1]
    expected_crc = calculate_crc(msg_type, payload)

    if received_crc != expected_crc:
        raise ValueError(
            f"CRC mismatch: expected 0x{expected_crc:02X}, "
            f"got 0x{received_crc:02X}"
        )

    return msg_type, bytes(payload)


# ---------------------------------------------------------------------------
# Stream frame parser
# ---------------------------------------------------------------------------

class FrameParser:
    """Streaming COBS frame parser — feed bytes, get decoded frames.

    State machine:
      WAIT_LEN_HI -> WAIT_LEN_LO -> READING -> WAIT_TERM
    """

    def __init__(self):
        self._state = 'WAIT_LEN_HI'
        self._len_hi = 0
        self._expected_len = 0
        self._buf = bytearray()
        self.frames_received = 0
        self.crc_errors = 0

    def feed(self, data: bytes) -> list:
        """Feed raw bytes from serial port, return list of (msg_type, payload)."""
        results = []

        for byte in data:
            if self._state == 'WAIT_LEN_HI':
                self._len_hi = byte
                self._state = 'WAIT_LEN_LO'

            elif self._state == 'WAIT_LEN_LO':
                self._expected_len = (self._len_hi << 8) | byte
                if self._expected_len == 0 or self._expected_len > MAX_FRAME_SIZE:
                    # Invalid length — resync
                    self._state = 'WAIT_LEN_HI'
                    continue
                self._buf.clear()
                self._state = 'READING'

            elif self._state == 'READING':
                self._buf.append(byte)
                if len(self._buf) >= self._expected_len:
                    self._state = 'WAIT_TERM'

            elif self._state == 'WAIT_TERM':
                if byte == 0x00:
                    # Got complete frame — validate
                    try:
                        msg_type, payload = validate_frame(bytes(self._buf))
                        self.frames_received += 1
                        results.append((msg_type, payload))
                    except ValueError:
                        self.crc_errors += 1
                else:
                    # Missing terminator — corrupted frame
                    self.crc_errors += 1
                self._state = 'WAIT_LEN_HI'

        return results

    def reset(self):
        """Reset parser state."""
        self._buf.clear()
        self._state = 'WAIT_LEN_HI'


# ---------------------------------------------------------------------------
# Dock -> Rover: frame builders
# ---------------------------------------------------------------------------

def encode_identify_request(
    dock_id: str, coven_name: str, assigned_name: str = ""
) -> bytes:
    """Build IDENTIFY_REQUEST frame (dock -> rover).

    Matches parse_identify_request() in dock_uart.rs.
    """
    payload = bytearray()

    dock_id_bytes = dock_id.encode('utf-8')
    payload.append(len(dock_id_bytes))
    payload.extend(dock_id_bytes)

    coven_bytes = coven_name.encode('utf-8')
    payload.append(len(coven_bytes))
    payload.extend(coven_bytes)

    name_bytes = assigned_name.encode('utf-8')
    payload.append(len(name_bytes))
    payload.extend(name_bytes)

    return build_frame(MSG_IDENTIFY_REQUEST, bytes(payload))


def encode_identify_ack(
    dock_id: str, assigned_name: str, message: str = "Welcome"
) -> bytes:
    """Build IDENTIFY_ACK frame (dock -> rover).

    Sent after receiving IDENTIFY_REPLY to confirm the rover's final name.
    Matches parse_identify_ack() in dock_uart.rs.
    """
    payload = bytearray()

    dock_id_bytes = dock_id.encode('utf-8')
    payload.append(len(dock_id_bytes))
    payload.extend(dock_id_bytes)

    name_bytes = assigned_name.encode('utf-8')
    payload.append(len(name_bytes))
    payload.extend(name_bytes)

    msg_bytes = message.encode('utf-8')
    payload.append(len(msg_bytes))
    payload.extend(msg_bytes)

    return build_frame(MSG_IDENTIFY_ACK, bytes(payload))


def encode_verify_ok(dock_id: str, module_id: str) -> bytes:
    """Build VERIFY_OK frame (dock -> rover).

    Matches parse_verify_ok() in dock_uart.rs.
    """
    payload = bytearray()

    dock_id_bytes = dock_id.encode('utf-8')
    payload.append(len(dock_id_bytes))
    payload.extend(dock_id_bytes)

    module_bytes = module_id.encode('utf-8')
    payload.append(len(module_bytes))
    payload.extend(module_bytes)

    return build_frame(MSG_VERIFY_OK, bytes(payload))


def encode_verify_fail(dock_id: str, module_id: str) -> bytes:
    """Build VERIFY_FAIL frame (dock -> rover).

    Same payload format as VERIFY_OK but different message type.
    """
    payload = bytearray()

    dock_id_bytes = dock_id.encode('utf-8')
    payload.append(len(dock_id_bytes))
    payload.extend(dock_id_bytes)

    module_bytes = module_id.encode('utf-8')
    payload.append(len(module_bytes))
    payload.extend(module_bytes)

    return build_frame(MSG_VERIFY_FAIL, bytes(payload))


def encode_task_request(task_data: dict) -> bytes:
    """Build DATA_FRAME/TASK_MESSAGE frame (dock -> rover).

    Matches parse_data_frame() subtype 0x10 in dock_uart.rs.
    Task data is JSON-encoded within the binary frame.
    """
    json_bytes = json.dumps(task_data).encode('utf-8')
    payload = bytes([SUBTYPE_TASK_MESSAGE]) + json_bytes
    return build_frame(MSG_DATA_FRAME, payload)


def encode_cmd_vel(linear: float, angular: float) -> bytes:
    """Build DATA_FRAME/CMD_VEL frame (dock -> rover).

    Matches parse_data_frame() subtype 0x30 in dock_uart.rs.
    Velocities encoded as little-endian f32.
    """
    payload = bytes([SUBTYPE_CMD_VEL]) + struct.pack('<ff', linear, angular)
    return build_frame(MSG_DATA_FRAME, payload)


def encode_system_ping() -> bytes:
    """Build SYSTEM_PING frame (bidirectional)."""
    return build_frame(MSG_SYSTEM_PING, b'')


# ---------------------------------------------------------------------------
# Rover -> Dock: payload decoders
# ---------------------------------------------------------------------------

def decode_identify_reply(payload: bytes) -> Optional[dict]:
    """Decode IDENTIFY_REPLY payload from rover.

    Matches encode_message() IdentifyRep in dock_uart.rs:
    "COV" magic(3) + type_byte(1) + rev_byte(1)
    + id_len(1) + module_id + battery(1) + status_len(1) + status
    """
    if len(payload) < 6:
        return None

    # Verify magic
    if payload[:3] != b'COV':
        return None

    type_byte = payload[3]
    rev_byte = payload[4]

    pos = 5

    # Module ID
    id_len = payload[pos]
    pos += 1
    if pos + id_len > len(payload):
        return None
    module_id = payload[pos:pos + id_len].decode('utf-8', errors='replace')
    pos += id_len

    # Battery level (0-100 as byte)
    battery_level = payload[pos] if pos < len(payload) else 100
    pos += 1

    # Status string
    if pos < len(payload):
        status_len = payload[pos]
        pos += 1
        status = payload[pos:pos + status_len].decode('utf-8', errors='replace')
    else:
        status = "OK"

    module_type = MODULE_TYPE_MAP.get(type_byte, "Unknown")

    # Capabilities byte (optional — appended by newer firmware)
    if pos < len(payload):
        capabilities = payload[pos]
    else:
        capabilities = _DEFAULT_CAPABILITIES.get(module_type, CAP_ENCODERS)

    return {
        "type": "IDENTIFY_REPLY",
        "module_id": module_id,
        "module_type": module_type,
        "firmware": f"{rev_byte}.0.0",
        "battery_level": float(battery_level),
        "status": status,
        "capabilities": capabilities,
    }


def decode_heartbeat(payload: bytes) -> Optional[dict]:
    """Decode MODULE_HEARTBEAT payload from rover.

    Matches encode_message() Heartbeat in dock_uart.rs:
    id_len(1) + module_id + battery(1) + status_byte(1)
    + x_mm(i32 LE) + y_mm(i32 LE) + theta_mrad(i16 LE)
    """
    if len(payload) < 1:
        return None

    pos = 0

    # Module ID
    id_len = payload[pos]
    pos += 1
    if pos + id_len > len(payload):
        return None
    module_id = payload[pos:pos + id_len].decode('utf-8', errors='replace')
    pos += id_len

    if pos + 12 > len(payload):
        return None

    battery_pct = payload[pos]
    pos += 1

    status_byte = payload[pos]
    pos += 1

    x_mm = struct.unpack_from('<i', payload, pos)[0]
    pos += 4
    y_mm = struct.unpack_from('<i', payload, pos)[0]
    pos += 4
    theta_mrad = struct.unpack_from('<h', payload, pos)[0]

    mission_status = MISSION_STATUS_MAP.get(status_byte, "UNKNOWN")

    return {
        "type": "HEARTBEAT",
        "module_id": module_id,
        "battery_pct": float(battery_pct),
        "mission_status": mission_status,
        "x": x_mm / 1000.0,
        "y": y_mm / 1000.0,
        "theta": theta_mrad / 1000.0,
    }


def decode_data_frame(payload: bytes) -> Optional[dict]:
    """Decode DATA_FRAME payload (dispatches on subtype byte).

    Matches encode_message() in dock_uart.rs for VerifyRep, Task*, and
    DataBatch/ScanData/OdomData messages.
    """
    if len(payload) < 1:
        return None

    subtype = payload[0]
    data = payload[1:]

    if subtype == SUBTYPE_VERIFY_REP:
        return _decode_verify_rep(data)
    elif subtype == SUBTYPE_TASK_MESSAGE:
        return _decode_task_json(data)
    elif subtype == SUBTYPE_SENSOR_DATA:
        return _decode_sensor_json(data)
    elif subtype == SUBTYPE_SENSOR_BATCH_CHUNK:
        return {"type": "SENSOR_BATCH_CHUNK", "chunk_data": bytes(data)}
    else:
        return None


def _decode_verify_rep(data: bytes) -> Optional[dict]:
    """Decode VERIFY_REP from DATA_FRAME subtype 0x01.

    id_len(1) + module_id + success(1) + failed_count(1)
    + [check_len(1) + check_str]... + note_len(1) + note
    """
    if len(data) < 3:
        return None

    pos = 0

    # Module ID
    id_len = data[pos]
    pos += 1
    module_id = data[pos:pos + id_len].decode('utf-8', errors='replace')
    pos += id_len

    # Success flag
    success = data[pos] != 0
    pos += 1

    # Failed checks
    failed_checks = []
    if pos < len(data):
        check_count = data[pos]
        pos += 1
        for _ in range(min(check_count, 10)):
            if pos >= len(data):
                break
            check_len = data[pos]
            pos += 1
            check_str = data[pos:pos + check_len].decode('utf-8', errors='replace')
            pos += check_len
            failed_checks.append(check_str)

    # Note
    note = ""
    if pos < len(data):
        note_len = data[pos]
        pos += 1
        note = data[pos:pos + note_len].decode('utf-8', errors='replace')

    return {
        "type": "VERIFY_REP",
        "module_id": module_id,
        "success": success,
        "failed_checks": failed_checks,
        "note": note,
    }


def _decode_task_json(data: bytes) -> Optional[dict]:
    """Decode TASK_MESSAGE (JSON within DATA_FRAME subtype 0x10)."""
    try:
        msg = json.loads(data.decode('utf-8'))
        # Tag with type for dispatch
        if "TaskAck" in msg:
            inner = msg["TaskAck"]
            return {"type": "TASK_ACK", **inner}
        elif "TaskStart" in msg:
            inner = msg["TaskStart"]
            return {"type": "TASK_START", **inner}
        elif "TaskComplete" in msg:
            inner = msg["TaskComplete"]
            return {"type": "TASK_COMPLETE", **inner}
        else:
            # Try direct fields
            return {"type": "TASK_MESSAGE", **msg}
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None


def _decode_sensor_json(data: bytes) -> Optional[dict]:
    """Decode SENSOR_DATA (JSON within DATA_FRAME subtype 0x20)."""
    try:
        msg = json.loads(data.decode('utf-8'))
        if "DataBatch" in msg:
            return {"type": "DATA_BATCH", **msg}
        elif "ScanData" in msg:
            return {"type": "SCAN_DATA", **msg}
        elif "OdomData" in msg:
            return {"type": "ODOM_DATA", **msg}
        else:
            return {"type": "SENSOR_DATA", **msg}
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None


# ---------------------------------------------------------------------------
# Top-level decode dispatcher
# ---------------------------------------------------------------------------

def decode_message(msg_type: int, payload: bytes) -> Optional[dict]:
    """Decode a received frame into a message dict.

    This is the main entry point for the dock side after FrameParser
    extracts (msg_type, payload) pairs.
    """
    if msg_type == MSG_IDENTIFY_REPLY:
        return decode_identify_reply(payload)
    elif msg_type == MSG_MODULE_HEARTBEAT:
        return decode_heartbeat(payload)
    elif msg_type == MSG_DATA_FRAME:
        return decode_data_frame(payload)
    elif msg_type == MSG_FAULT_ALERT:
        return {"type": "FAULT_ALERT", "payload": payload.hex()}
    elif msg_type == MSG_SYSTEM_PING:
        return {"type": "SYSTEM_PING"}
    else:
        return None


# ---------------------------------------------------------------------------
# Binary batch chunk reassembly
# ---------------------------------------------------------------------------

class BatchChunkAssembler:
    """Reassembles binary sensor batch chunks into a complete batch dict.

    The transport layer is sensor-agnostic — sensor_config and sensor_data
    are treated as opaque bytes. Only downstream processors interpret them.

    Lifecycle per batch:
      1. BATCH_HEADER chunk → stores metadata, allocates chunk buffer
      2. BATCH_DATA chunks → unpacks samples, stores by sequence number
      3. All chunks received → returns complete batch dict
    """

    def __init__(self, timeout_secs: float = 30.0):
        self._timeout_secs = timeout_secs
        self.reset()

    def reset(self):
        """Clear all state for a new batch transfer."""
        self._header: Optional[dict] = None
        self._chunks: Dict[int, list] = {}
        self._last_chunk_time: float = 0.0

    def feed_chunk(self, chunk_data: bytes) -> Optional[dict]:
        """Feed a SENSOR_BATCH_CHUNK payload (after subtype byte removed).

        Returns the complete batch dict when all chunks are received,
        or None if still accumulating.
        """
        if len(chunk_data) < 1:
            return None

        # Check for timeout on partial batch
        if self._header and self._last_chunk_time > 0:
            elapsed = time.monotonic() - self._last_chunk_time
            if elapsed > self._timeout_secs:
                received = len(self._chunks)
                expected = self._header.get("total_chunks", 0)
                logger.warning(
                    "Batch transfer timed out after %.1fs "
                    "(%d/%d chunks received) — discarding",
                    elapsed, received, expected,
                )
                self.reset()

        chunk_type = chunk_data[0]
        data = chunk_data[1:]

        if chunk_type == CHUNK_TYPE_HEADER:
            return self._handle_header(data)
        elif chunk_type == CHUNK_TYPE_DATA:
            return self._handle_data(data)
        else:
            logger.warning("Unknown batch chunk type: 0x%02X", chunk_type)
            return None

    def _handle_header(self, data: bytes) -> Optional[dict]:
        """Process a BATCH_HEADER chunk."""
        header = self._decode_header(data)
        if header is None:
            logger.warning("Failed to decode batch header")
            return None

        # If we had a partial batch from a different transfer, discard it
        if self._header and self._header.get("batch_id") != header["batch_id"]:
            logger.warning(
                "New batch header (id=0x%08X) received while previous "
                "batch (id=0x%08X) was incomplete — discarding previous",
                header["batch_id"],
                self._header.get("batch_id", 0),
            )

        self._header = header
        self._chunks = {}
        self._last_chunk_time = time.monotonic()

        logger.info(
            "Batch header: module=%s mission=%s samples=%d chunks=%d sensor=0x%02X",
            header["module_id"], header["mission_id"],
            header["total_samples"], header["total_chunks"],
            header["sensor_type"],
        )

        # Edge case: empty batch (0 data chunks expected)
        if header["total_chunks"] == 0:
            return self._assemble_batch()

        return None

    def _handle_data(self, data: bytes) -> Optional[dict]:
        """Process a BATCH_DATA chunk."""
        if self._header is None:
            logger.warning("Received data chunk without header — dropping")
            return None

        result = self._decode_data_chunk(data)
        if result is None:
            logger.warning("Failed to decode data chunk")
            return None

        batch_id, chunk_seq, samples = result

        # Verify batch_id matches
        if batch_id != self._header["batch_id"]:
            logger.warning(
                "Data chunk batch_id 0x%08X doesn't match header 0x%08X — dropping",
                batch_id, self._header["batch_id"],
            )
            return None

        self._chunks[chunk_seq] = samples
        self._last_chunk_time = time.monotonic()

        # Check if all chunks received
        if len(self._chunks) >= self._header["total_chunks"]:
            return self._assemble_batch()

        return None

    def _decode_header(self, data: bytes) -> Optional[dict]:
        """Decode BATCH_HEADER fields from binary data."""
        if len(data) < 8:
            return None

        pos = 0
        batch_id = struct.unpack_from('<I', data, pos)[0]; pos += 4
        total_samples = struct.unpack_from('<H', data, pos)[0]; pos += 2
        total_chunks = struct.unpack_from('<H', data, pos)[0]; pos += 2

        # Module ID (length-prefixed)
        if pos >= len(data):
            return None
        mid_len = data[pos]; pos += 1
        if pos + mid_len > len(data):
            return None
        module_id = data[pos:pos + mid_len].decode('utf-8', errors='replace')
        pos += mid_len

        # Mission ID (length-prefixed)
        if pos >= len(data):
            return None
        msn_len = data[pos]; pos += 1
        if pos + msn_len > len(data):
            return None
        mission_id = data[pos:pos + msn_len].decode('utf-8', errors='replace')
        pos += msn_len

        # Batch metadata
        if pos + 14 > len(data):
            return None
        mission_start = struct.unpack_from('<d', data, pos)[0]; pos += 8
        wheel_radius_mm = struct.unpack_from('<H', data, pos)[0]; pos += 2
        wheel_base_mm = struct.unpack_from('<H', data, pos)[0]; pos += 2
        ticks_per_rev = struct.unpack_from('<H', data, pos)[0]; pos += 2

        # Sensor type + opaque config blob
        if pos + 3 > len(data):
            return None
        sensor_type = data[pos]; pos += 1
        config_len = struct.unpack_from('<H', data, pos)[0]; pos += 2
        if pos + config_len > len(data):
            return None
        sensor_config = bytes(data[pos:pos + config_len])

        return {
            "batch_id": batch_id,
            "total_samples": total_samples,
            "total_chunks": total_chunks,
            "module_id": module_id,
            "mission_id": mission_id,
            "mission_start": mission_start,
            "wheel_radius_mm": wheel_radius_mm,
            "wheel_base_mm": wheel_base_mm,
            "ticks_per_rev": ticks_per_rev,
            "sensor_type": sensor_type,
            "sensor_config": sensor_config,
        }

    def _decode_data_chunk(self, data: bytes) -> Optional[tuple]:
        """Decode BATCH_DATA chunk. Returns (batch_id, chunk_seq, samples)."""
        if len(data) < 8:
            return None

        pos = 0
        batch_id = struct.unpack_from('<I', data, pos)[0]; pos += 4
        chunk_seq = struct.unpack_from('<H', data, pos)[0]; pos += 2
        sample_count = struct.unpack_from('<H', data, pos)[0]; pos += 2

        samples = []
        for _ in range(sample_count):
            # Fixed fields: timestamp(8) + left(4) + right(4) + data_len(2) = 18
            if pos + 18 > len(data):
                break

            timestamp = struct.unpack_from('<d', data, pos)[0]; pos += 8
            left_ticks = struct.unpack_from('<i', data, pos)[0]; pos += 4
            right_ticks = struct.unpack_from('<i', data, pos)[0]; pos += 4
            sensor_data_len = struct.unpack_from('<H', data, pos)[0]; pos += 2

            if pos + sensor_data_len > len(data):
                break
            sensor_data = bytes(data[pos:pos + sensor_data_len])
            pos += sensor_data_len

            samples.append({
                "timestamp": timestamp,
                "left_ticks": left_ticks,
                "right_ticks": right_ticks,
                "sensor_data": sensor_data,
            })

        return (batch_id, chunk_seq, samples)

    def _assemble_batch(self) -> dict:
        """Assemble all received chunks into the final batch dict."""
        header = self._header

        # Concatenate samples in sequence order
        all_samples = []
        for seq in sorted(self._chunks.keys()):
            all_samples.extend(self._chunks[seq])

        result = {
            "type": "DATA_BATCH",
            "module_id": header["module_id"],
            "mission_id": header["mission_id"],
            "sensor_type": header["sensor_type"],
            "sensor_config": header["sensor_config"],
            "batch": {
                "mission_start": header["mission_start"],
                "wheel_radius_mm": header["wheel_radius_mm"],
                "wheel_base_mm": header["wheel_base_mm"],
                "ticks_per_rev": header["ticks_per_rev"],
                "samples": all_samples,
            },
        }

        logger.info(
            "Batch assembled: %d samples from %d chunks (sensor=0x%02X)",
            len(all_samples), len(self._chunks), header["sensor_type"],
        )

        self.reset()
        return result
