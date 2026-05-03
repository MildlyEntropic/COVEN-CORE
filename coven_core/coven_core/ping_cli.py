#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
ping_cli.py — `coven-ping`: send a SYSTEM_PING over a serial port and
wait for a response.

This is a no-ROS2 connectivity check. It opens a serial device (or PTY
slave path), writes a single SYSTEM_PING (0xFF) frame, and reads any
incoming frames for a configurable timeout. Useful for:

  * Verifying the sim_rover_proxy's PTY is reachable before booting the
    full dock stack.
  * Smoke-testing a real Pi Zero rover over USB before launching ROS2.
  * Confirming COBS framing survives a particular cable / virtual transport.

Usage:
    coven-ping /tmp/coven_sim_uart
    coven-ping /dev/ttyACM0 --baud 115200 --timeout 3.0
    coven-ping /dev/ttyACM0 --count 5

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Tuple

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None  # type: ignore

# rclpy is not needed for this CLI — it's a pure serial tool. Importing
# from coven_core.frame_codec only requires Python stdlib.
from coven_core.frame_codec import FrameParser, build_frame


MSG_SYSTEM_PING = 0xFF


def _format_frame(msg_type: int, payload: bytes) -> str:
    """Render a parsed frame as a short label for logging."""
    label = {
        0x01: "IDENTIFY_REQUEST",
        0x02: "IDENTIFY_REPLY",
        0x03: "VERIFY_OK",
        0x04: "VERIFY_FAIL",
        0x05: "IDENTIFY_ACK",
        0x10: "DATA_FRAME",
        0x20: "MODULE_HEARTBEAT",
        0x30: "FAULT_ALERT",
        0xFF: "SYSTEM_PING",
    }.get(msg_type, f"UNKNOWN(0x{msg_type:02x})")
    return f"{label} ({len(payload)} bytes)"


def _open_serial(port: str, baud: int):
    if serial is None:
        raise RuntimeError(
            "pyserial is not installed. Inside the sim Docker container "
            "this is preinstalled; on a host you can `pip3 install pyserial`."
        )
    return serial.Serial(port=port, baudrate=baud, timeout=0.1)


def ping_once(
    port: str,
    baud: int = 115200,
    timeout: float = 2.0,
) -> Tuple[bool, List[Tuple[int, bytes]]]:
    """Send one SYSTEM_PING and return (ok, list_of_frames_received).

    `ok` is True if at least one frame came back within `timeout` seconds.
    Note that the COVEN protocol does not require an explicit PING reply —
    a real rover or sim proxy may emit a heartbeat or other frame on its
    own cadence; any received frame counts as evidence the link is alive.
    """
    ser = _open_serial(port, baud)
    parser = FrameParser()
    received: List[Tuple[int, bytes]] = []
    try:
        # Send the ping (empty payload).
        frame = build_frame(MSG_SYSTEM_PING, b"")
        ser.write(frame)
        ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            data = ser.read(256)
            if data:
                received.extend(parser.feed(data))
                # As soon as we see anything, we're done.
                if received:
                    break
        return (len(received) > 0, received)
    finally:
        ser.close()


def _build_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="coven-ping",
        description=(
            "Send a SYSTEM_PING to a COVEN device over serial and report "
            "any frames received. Works against real Pi Zero rovers on "
            "/dev/ttyACM* and against sim rover proxies on a PTY symlink."
        ),
    )
    parser.add_argument(
        "port",
        help="Serial device path (e.g. /dev/ttyACM0 or /tmp/coven_sim_uart).",
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate. Default 115200 (the COVEN protocol's standard rate).",
    )
    parser.add_argument(
        "--timeout", type=float, default=2.0,
        help="Per-ping read timeout in seconds. Default 2.0.",
    )
    parser.add_argument(
        "--count", type=int, default=1,
        help="Number of pings to send. Default 1.",
    )
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="Seconds between pings (when --count > 1). Default 0.5.",
    )
    return parser


def main(argv=None) -> int:
    args = _build_args().parse_args(argv)

    success_count = 0
    for i in range(args.count):
        try:
            ok, frames = ping_once(args.port, args.baud, args.timeout)
        except Exception as e:
            print(f"[{i + 1}/{args.count}] ERROR opening {args.port}: {e}",
                  file=sys.stderr)
            return 2

        if ok:
            success_count += 1
            labels = ", ".join(_format_frame(t, p) for t, p in frames)
            print(f"[{i + 1}/{args.count}] OK from {args.port}: {labels}")
        else:
            print(f"[{i + 1}/{args.count}] no response from {args.port} "
                  f"within {args.timeout:.1f}s")

        if i + 1 < args.count:
            time.sleep(args.interval)

    print(f"\n{success_count}/{args.count} pings answered.")
    return 0 if success_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
