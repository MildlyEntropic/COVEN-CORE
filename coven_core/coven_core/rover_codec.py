#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
rover_codec.py — Python encoders for rover-to-dock COVEN messages.

The Rust firmware (coven_core/rover/src/dock_uart.rs) implements all
rover→dock message encoding. The dock-side Python codec in frame_codec.py
implements all dock→rover encoding plus the rover→dock *decoders*. The
rover-side encoders were never needed in Python because the production
rover is Rust.

This module supplies them so the simulation rover-proxy can speak the
exact same wire format the Rust firmware uses, byte-for-byte. The proxy
subscribes to Gazebo's simulated sensor topics and re-encodes that data
through this codec, so the dock cannot distinguish a sim rover from a
physical Pi Zero rover at the protocol layer.

Wire format (per Interface Specification v0.3 — COBS):
    [LEN_HI] [LEN_LO] [COBS-encoded data] [0x00]
    Inside COBS block: [TYPE] [PAYLOAD] [CRC]
    CRC = XOR of TYPE and all PAYLOAD bytes.

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import json
import struct
from typing import Iterable

from .frame_codec import (
    SUBTYPE_SENSOR_DATA,
    SUBTYPE_TASK_MESSAGE,
    SUBTYPE_VERIFY_REP,
    build_frame,
)


# ---------------------------------------------------------------------------
# Message type constants (per Interface Specification v0.3)
# ---------------------------------------------------------------------------

MSG_IDENTIFY_REPLY = 0x02
MSG_DATA_FRAME = 0x10
MSG_MODULE_HEARTBEAT = 0x20
MSG_FAULT_ALERT = 0x30
MSG_SYSTEM_PING = 0xFF


# ---------------------------------------------------------------------------
# Module type byte (in IDENTIFY_REPLY)
#
# Mirrors the Rust firmware's match table in dock_uart.rs::encode_message
# under the IdentifyRep arm. Unknown classes resolve to 0x00.
# ---------------------------------------------------------------------------

MODULE_TYPE_BYTE = {
    "ReconRover": 0x01,
    "CargoRover": 0x02,
    "DrillRover": 0x03,
    "ArduinoBot": 0x04,
}


# ---------------------------------------------------------------------------
# Mission status byte (in MODULE_HEARTBEAT)
# ---------------------------------------------------------------------------

MISSION_STATUS_BYTE = {
    "IDLE": 0x00,
    "ACTIVE": 0x01,
    "STARTUP": 0x02,
    "RETURNING": 0x03,
}


# ---------------------------------------------------------------------------
# IDENTIFY_REPLY (rover → dock, message type 0x02)
# ---------------------------------------------------------------------------

def encode_identify_reply(
    module_id: str,
    module_type: str,
    firmware: str,
    battery_pct: float,
    status: str,
    capabilities: int,
) -> bytes:
    """Encode an IDENTIFY_REPLY payload and wrap it in a complete UART frame.

    Wire layout (matches Rust dock_uart::encode_message::IdentifyRep arm):
        b"COV"                             3 bytes  magic
        type_byte                          1 byte   from MODULE_TYPE_BYTE
        revision                           1 byte   firmware major version
        id_len                             1 byte
        module_id                          id_len bytes
        battery                            1 byte   clamped [0,100]
        status_len                         1 byte   capped at 255
        status                             status_len bytes
        capabilities                       1 byte   bitmask
    """
    payload = bytearray(b"COV")
    payload.append(MODULE_TYPE_BYTE.get(module_type, 0x00))

    # Revision: parse the major version from a "X.Y.Z"-style firmware string.
    try:
        rev = int(firmware.split(".", 1)[0])
    except (ValueError, IndexError):
        rev = 0
    payload.append(rev & 0xFF)

    id_bytes = module_id.encode("utf-8")
    payload.append(len(id_bytes) & 0xFF)
    payload.extend(id_bytes)

    payload.append(int(max(0.0, min(100.0, battery_pct))))

    status_bytes = status.encode("utf-8")
    status_len = min(len(status_bytes), 255)
    payload.append(status_len)
    payload.extend(status_bytes[:status_len])

    payload.append(capabilities & 0xFF)

    return build_frame(MSG_IDENTIFY_REPLY, bytes(payload))


# ---------------------------------------------------------------------------
# VERIFY_REP (rover → dock, DATA_FRAME subtype 0x01)
# ---------------------------------------------------------------------------

def encode_verify_rep(
    module_id: str,
    success: bool,
    failed_checks: Iterable[str],
    note: str,
) -> bytes:
    """Encode a VERIFY_REP as a DATA_FRAME with subtype 0x01.

    Wire layout (matches Rust VerifyRep arm):
        subtype 0x01                       1 byte
        id_len                             1 byte
        module_id                          id_len bytes
        success_flag                       1 byte (0 or 1)
        failed_count                       1 byte (capped at 10)
        per check: check_len(1) + check_bytes (each check capped at 32 bytes)
        note_len                           1 byte (capped at 100)
        note                               note_len bytes
    """
    payload = bytearray([SUBTYPE_VERIFY_REP])

    id_bytes = module_id.encode("utf-8")
    payload.append(len(id_bytes) & 0xFF)
    payload.extend(id_bytes)

    payload.append(1 if success else 0)

    checks = list(failed_checks)
    payload.append(min(len(checks), 255))
    for check in checks[:10]:  # match Rust's take(10)
        check_bytes = check.encode("utf-8")
        cap = min(len(check_bytes), 32)
        payload.append(cap)
        payload.extend(check_bytes[:cap])

    note_bytes = note.encode("utf-8")
    note_cap = min(len(note_bytes), 100)
    payload.append(note_cap)
    payload.extend(note_bytes[:note_cap])

    return build_frame(MSG_DATA_FRAME, bytes(payload))


# ---------------------------------------------------------------------------
# MODULE_HEARTBEAT (rover → dock, message type 0x20)
# ---------------------------------------------------------------------------

def encode_heartbeat(
    module_id: str,
    battery_pct: float,
    mission_status: str,
    x: float,
    y: float,
    theta: float,
) -> bytes:
    """Encode a MODULE_HEARTBEAT payload and wrap it in a UART frame.

    Wire layout (matches Rust Heartbeat arm):
        id_len                             1 byte
        module_id                          id_len bytes
        battery                            1 byte clamped [0, 100]
        mission_status_byte                1 byte from MISSION_STATUS_BYTE (0xFF on unknown)
        x_mm                               i32 little-endian (position * 1000)
        y_mm                               i32 little-endian
        theta_mrad                         i16 little-endian (heading * 1000)
    """
    payload = bytearray()

    id_bytes = module_id.encode("utf-8")
    payload.append(len(id_bytes) & 0xFF)
    payload.extend(id_bytes)

    payload.append(int(max(0.0, min(100.0, battery_pct))))
    payload.append(MISSION_STATUS_BYTE.get(mission_status, 0xFF))

    payload.extend(struct.pack("<i", int(x * 1000.0)))
    payload.extend(struct.pack("<i", int(y * 1000.0)))
    payload.extend(struct.pack("<h", int(theta * 1000.0)))

    return build_frame(MSG_MODULE_HEARTBEAT, bytes(payload))


# ---------------------------------------------------------------------------
# Sensor data streams (rover → dock, DATA_FRAME subtype 0x20, JSON payload)
#
# These mirror the Rust ScanData / OdomData / TaskAck / TaskStart /
# TaskComplete RoverMessage variants, which serialize as serde JSON inside
# DATA_FRAME with subtype 0x20 (sensor) or 0x10 (task).
#
# The dock decodes them via _decode_sensor_json / _decode_task_json, which
# look for keys "ScanData", "OdomData", "DataBatch", "TaskAck", etc.
# ---------------------------------------------------------------------------

def encode_scan_data_json(
    module_id: str,
    timestamp: float,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    ranges_mm: Iterable[int],
) -> bytes:
    """Encode a real-time ScanData message as DATA_FRAME subtype 0x20 JSON."""
    obj = {
        "ScanData": {
            "module_id": module_id,
            "scan": {
                "timestamp": timestamp,
                "angle_min": angle_min,
                "angle_max": angle_max,
                "angle_increment": angle_increment,
                "range_min": range_min,
                "range_max": range_max,
                "ranges_mm": list(ranges_mm),
            },
        }
    }
    json_bytes = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    payload = bytes([SUBTYPE_SENSOR_DATA]) + json_bytes
    return build_frame(MSG_DATA_FRAME, payload)


def encode_odom_data_json(
    module_id: str,
    timestamp: float,
    x: float,
    y: float,
    theta: float,
    v_linear: float,
    v_angular: float,
) -> bytes:
    """Encode a real-time OdomData message as DATA_FRAME subtype 0x20 JSON."""
    obj = {
        "OdomData": {
            "module_id": module_id,
            "odom": {
                "timestamp": timestamp,
                "x": x,
                "y": y,
                "theta": theta,
                "v_linear": v_linear,
                "v_angular": v_angular,
            },
        }
    }
    json_bytes = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    payload = bytes([SUBTYPE_SENSOR_DATA]) + json_bytes
    return build_frame(MSG_DATA_FRAME, payload)


def encode_task_ack_json(module_id: str, task_id: str, success: bool) -> bytes:
    """Encode a TaskAck message as DATA_FRAME subtype 0x10 JSON."""
    obj = {
        "TaskAck": {
            "module_id": module_id,
            "task_id": task_id,
            "success": success,
        }
    }
    json_bytes = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    payload = bytes([SUBTYPE_TASK_MESSAGE]) + json_bytes
    return build_frame(MSG_DATA_FRAME, payload)


def encode_task_start_json(module_id: str, task_id: str, timestamp: float) -> bytes:
    """Encode a TaskStart message as DATA_FRAME subtype 0x10 JSON."""
    obj = {
        "TaskStart": {
            "module_id": module_id,
            "task_id": task_id,
            "timestamp": timestamp,
        }
    }
    json_bytes = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    payload = bytes([SUBTYPE_TASK_MESSAGE]) + json_bytes
    return build_frame(MSG_DATA_FRAME, payload)


def encode_task_complete_json(
    module_id: str,
    task_id: str,
    success: bool,
    coverage: float,
    duration: float,
) -> bytes:
    """Encode a TaskComplete message as DATA_FRAME subtype 0x10 JSON."""
    obj = {
        "TaskComplete": {
            "module_id": module_id,
            "task_id": task_id,
            "success": success,
            "map_data": "",
            "coverage": coverage,
            "duration": duration,
        }
    }
    json_bytes = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    payload = bytes([SUBTYPE_TASK_MESSAGE]) + json_bytes
    return build_frame(MSG_DATA_FRAME, payload)
