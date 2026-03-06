"""
frame_codec.py — COVEN binary frame codec for dock-rover communication.

Mirrors the Rust implementation in rover/src/dock_uart.rs exactly.
The dock Python side uses this to build/parse the same binary frames
that the Rust rover firmware sends/receives over UART.

Frame format (per Interface Specification v0.2):
  [START] [TYPE] [LEN] [PAYLOAD] [CRC] [END]
  - START: 0x7E
  - TYPE:  Message type byte (0x01-0xFF)
  - LEN:   Payload length (1 byte, max 255)
  - PAYLOAD: Variable length data
  - CRC:   8-bit checksum (XOR of TYPE, LEN, and PAYLOAD)
  - END:   0x7F

Author: Alexander Shultis
Date: March 2026
"""

import json
import struct
from typing import List, Optional


# --- Frame constants ---

FRAME_START = 0x7E
FRAME_END = 0x7F
MAX_PAYLOAD_SIZE = 255


# --- Message types from Interface Specification v0.2 ---

MSG_IDENTIFY_REQUEST = 0x01   # dock -> rover
MSG_IDENTIFY_REPLY = 0x02     # rover -> dock
MSG_VERIFY_OK = 0x03          # dock -> rover
MSG_VERIFY_FAIL = 0x04        # dock -> rover
MSG_DATA_FRAME = 0x10         # bidirectional
MSG_MODULE_HEARTBEAT = 0x20   # rover -> dock
MSG_FAULT_ALERT = 0x30        # rover -> dock
MSG_SYSTEM_PING = 0xFF        # bidirectional

# Data frame subtypes (first byte of DATA_FRAME payload)
SUBTYPE_VERIFY_REP = 0x01
SUBTYPE_TASK_MESSAGE = 0x10
SUBTYPE_SENSOR_DATA = 0x20
SUBTYPE_CMD_VEL = 0x30
SUBTYPE_ENABLE_POWER = 0x40

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
# Low-level frame operations
# ---------------------------------------------------------------------------

def calculate_crc(msg_type: int, payload: bytes) -> int:
    """Calculate CRC-8 (XOR of type, length, and all payload bytes)."""
    crc = msg_type ^ len(payload)
    for b in payload:
        crc ^= b
    return crc & 0xFF


def build_frame(msg_type: int, payload: bytes) -> bytes:
    """Build a complete binary frame from message type and payload."""
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"Payload too large: {len(payload)} bytes (max {MAX_PAYLOAD_SIZE})"
        )
    crc = calculate_crc(msg_type, payload)
    return bytes([
        FRAME_START,
        msg_type,
        len(payload),
        *payload,
        crc,
        FRAME_END,
    ])


def validate_frame(frame: bytes) -> tuple:
    """Validate and extract (msg_type, payload) from a complete frame.

    Returns (msg_type, payload) on success, raises ValueError on failure.
    """
    if len(frame) < 5:
        raise ValueError(f"Frame too short: {len(frame)} bytes")
    if frame[0] != FRAME_START:
        raise ValueError(f"Invalid start byte: 0x{frame[0]:02X}")
    if frame[-1] != FRAME_END:
        raise ValueError(f"Invalid end byte: 0x{frame[-1]:02X}")

    msg_type = frame[1]
    payload_len = frame[2]
    expected_len = payload_len + 5  # start + type + len + payload + crc + end

    if len(frame) != expected_len:
        raise ValueError(
            f"Frame length mismatch: expected {expected_len}, got {len(frame)}"
        )

    payload = frame[3:3 + payload_len]
    received_crc = frame[3 + payload_len]
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
    """Streaming frame parser — feed bytes, get decoded frames.

    Handles partial reads, noise between frames, and buffer overflow.
    """

    def __init__(self):
        self._buf = bytearray()
        self._in_frame = False
        self.frames_received = 0
        self.crc_errors = 0

    def feed(self, data: bytes) -> list:
        """Feed raw bytes from serial port, return list of (msg_type, payload).

        Silently skips noise between frames and logs CRC errors.
        """
        results = []

        for byte in data:
            if byte == FRAME_START:
                self._buf.clear()
                self._buf.append(byte)
                self._in_frame = True
            elif self._in_frame:
                self._buf.append(byte)

                if byte == FRAME_END:
                    self._in_frame = False
                    try:
                        msg_type, payload = validate_frame(bytes(self._buf))
                        self.frames_received += 1
                        results.append((msg_type, payload))
                    except ValueError:
                        self.crc_errors += 1
                    self._buf.clear()

                # Prevent buffer overflow from malformed data
                if len(self._buf) > MAX_PAYLOAD_SIZE + 5:
                    self._buf.clear()
                    self._in_frame = False

        return results

    def reset(self):
        """Reset parser state."""
        self._buf.clear()
        self._in_frame = False


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
