#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
swarm_demo.py — 8-rover heterogeneous swarm bidding demo.

Spawns 8 mock rovers as Python threads, each owning a virtual UART (PTY
pair) that speaks the byte-exact COVEN wire format via coven_core.rover_codec
(verified against the dock's production decoders by 15 unit tests). Each
rover walks the full BOOT → IDENTIFY → WAIT_VERIFY → NORMAL state machine.

The swarm composition pins the polymorphism story:
    4 × LiDAR rovers       (caps 0x03)  — MappingRover analog
    2 × spectrometer rovers (caps 0x13) — SpectralRover analog
    2 × barometer rovers   (caps 0x41)  — BarometerRover analog

The demo dock then runs three auctions in succession, one per task type
(EXPLORE, SPECTRAL, BAROMETRIC). For every auction, every rover's bid is
printed — including 999999 for capability-incompatible rovers — and the
lowest-bid winner is dispatched. Other rovers stay docked and continue
heartbeating; the winner goes IN_FIELD silent until it returns with a
TASK_COMPLETE.

This is the visible-on-screen counterpart to test_polymorphism_swarm.py
(which runs 100 randomized auctions in an assert-only unit test). Same
algorithm, same names, same matrix — just rendered for a screenshot.

No ROS2, no Gazebo, no Docker. Pure Python.

Usage:
    coven-swarm-demo                # default: 60s of runtime
    coven-swarm-demo 90             # 90s, gives slow auctions room

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO))

from coven_core.frame_codec import (  # noqa: E402
    FrameParser,
    decode_data_frame,
    decode_heartbeat,
    encode_identify_ack,
    encode_identify_request,
    encode_task_request,
    encode_verify_ok,
)
from coven_core.naming import (  # noqa: E402
    get_coven_name,
    get_witch_name,
)
from coven_core.rover_codec import (  # noqa: E402
    encode_heartbeat,
    encode_identify_reply,
    encode_task_ack_json,
    encode_task_complete_json,
    encode_task_start_json,
    encode_verify_rep,
)
from coven_core.task_auctioneer import (  # noqa: E402
    Mission,
    PayloadType,
    RoverInfo,
    RoverStatus,
    TaskType,
)


# ---------------------------------------------------------------------------
# Color palette (Flat UI — same as demo_dock_for_screenshots.py for cohesion)
# ---------------------------------------------------------------------------

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"

PASS = "\033[38;2;46;204;113m"        # #2ECC71 Emerald
FAIL = "\033[38;2;231;76;60m"         # #E74C3C Alizarin
WARN = "\033[38;2;243;156;18m"        # #F39C12 Orange
COVEN_COLOR = "\033[38;2;41;128;185m" # #2980B9 Belize Hole
WITCH_COLOR = "\033[38;2;155;89;182m" # #9B59B6 Wisteria
CYAN = "\033[36m"
MAGENTA = "\033[35m"


def pretty(name: str) -> str:
    return name.replace("_", " ")


def coven(name: str) -> str:
    return f"{COVEN_COLOR}{BOLD}{pretty(name)}{RESET}"


def witch(name: str) -> str:
    return f"{WITCH_COLOR}{BOLD}{pretty(name)}{RESET}"


def ok(text: str) -> str:
    return f"{PASS}{BOLD}{text}{RESET}"


def bad(text: str) -> str:
    return f"{FAIL}{BOLD}{text}{RESET}"


def warn(text: str) -> str:
    return f"{WARN}{BOLD}{text}{RESET}"


def banner(text: str, color: str = CYAN) -> None:
    line = "═" * (len(text) + 4)
    print(f"\n{color}{BOLD}{line}{RESET}")
    print(f"{color}{BOLD}║ {text} ║{RESET}")
    print(f"{color}{BOLD}{line}{RESET}\n")


# ---------------------------------------------------------------------------
# Capability bit layout & swarm composition (canonical from naming.py)
# ---------------------------------------------------------------------------

CAP_ENCODERS = 0x01
CAP_LIDAR = 0x02
CAP_SPECTROMETER = 0x10
CAP_BAROMETER = 0x40

# (PayloadType, capability bitmask, count) — matches §4.5 of the thesis.
SWARM_COMPOSITION = [
    (PayloadType.LIDAR,        CAP_ENCODERS | CAP_LIDAR,                   4),
    (PayloadType.SPECTROMETER, CAP_ENCODERS | CAP_LIDAR | CAP_SPECTROMETER, 2),
    (PayloadType.BAROMETER,    CAP_ENCODERS | CAP_BAROMETER,                2),
]


def payload_label(p: PayloadType) -> str:
    return {
        PayloadType.LIDAR:        "LiDAR       ",
        PayloadType.SPECTROMETER: "Spectrometer",
        PayloadType.BAROMETER:    "Barometer   ",
    }.get(p, p.value)


# ---------------------------------------------------------------------------
# FakeRover — one Python thread per simulated rover.
# ---------------------------------------------------------------------------

@dataclass
class FakeRover:
    """A mock rover that owns one end of a PTY pair and speaks COVEN."""
    module_id: str
    payload: PayloadType
    capabilities: int
    master_fd: int  # dock side
    slave_fd: int   # rover side (kept open so writes from dock don't EIO)
    info: RoverInfo  # the dock's view of this rover (used for bidding)
    state: str = "BOOT"
    parser: FrameParser = field(default_factory=FrameParser)
    in_field: threading.Event = field(default_factory=threading.Event)
    field_complete: threading.Event = field(default_factory=threading.Event)
    current_task_id: Optional[str] = None
    field_started_at: Optional[float] = None
    field_duration: float = 4.0  # seconds in field before "returning"
    _stop: threading.Event = field(default_factory=threading.Event)

    def write_to_dock(self, frame: bytes) -> None:
        """Rover→Dock: write to the slave end (the dock reads from master)."""
        try:
            os.write(self.slave_fd, frame)
        except OSError:
            pass

    def read_from_dock(self, timeout: float = 0.1) -> List[Tuple[int, bytes]]:
        """Dock→Rover: read from slave; the dock writes to master."""
        r, _, _ = select.select([self.slave_fd], [], [], timeout)
        if not r:
            return []
        try:
            data = os.read(self.slave_fd, 4096)
        except OSError:
            return []
        return list(self.parser.feed(data))

    def run(self) -> None:
        """Main rover loop: handle messages, heartbeat, simulate field ops."""
        last_heartbeat = 0.0
        while not self._stop.is_set():
            now = time.time()

            # In-field: silent for `field_duration` seconds, then "return".
            if self.in_field.is_set():
                if self.field_started_at is None:
                    self.field_started_at = now
                if now - self.field_started_at >= self.field_duration:
                    # "Return" by re-engaging communications: send TASK_COMPLETE.
                    self.write_to_dock(encode_task_complete_json(
                        module_id=self.module_id,
                        task_id=self.current_task_id or "?",
                        success=True,
                        coverage=1.0,
                        duration=now - self.field_started_at,
                    ))
                    self.in_field.clear()
                    self.field_started_at = None
                    self.current_task_id = None
                    self.state = "NORMAL"
                    self.field_complete.set()
                # No heartbeats, no scan, no odom while in field.
                time.sleep(0.05)
                continue

            # Heartbeat at 1 Hz when docked and at least in WAIT_VERIFY.
            if self.state in ("WAIT_VERIFY", "NORMAL") and now - last_heartbeat >= 1.0:
                status = "IDLE" if self.state == "NORMAL" else "STARTUP"
                self.write_to_dock(encode_heartbeat(
                    module_id=self.module_id,
                    battery_pct=self.info.battery_pct,
                    mission_status=status,
                    x=self.info.position[0],
                    y=self.info.position[1],
                    theta=self.info.heading,
                ))
                last_heartbeat = now

            # Process any inbound dock messages.
            for msg_type, payload in self.read_from_dock(timeout=0.1):
                self._handle_dock_message(msg_type, payload)

    def _handle_dock_message(self, msg_type: int, payload: bytes) -> None:
        if msg_type == 0x01:  # IDENTIFY_REQUEST
            self.write_to_dock(encode_identify_reply(
                module_id=self.module_id,
                module_type="ReconRover",
                firmware="0.1.0",
                battery_pct=self.info.battery_pct,
                status="OK",
                capabilities=self.capabilities,
            ))
            self.state = "IDENTIFY"
        elif msg_type == 0x05:  # IDENTIFY_ACK
            self.state = "WAIT_VERIFY"
        elif msg_type == 0x03:  # VERIFY_OK
            self.write_to_dock(encode_verify_rep(
                module_id=self.module_id,
                success=True,
                failed_checks=[],
                note="All systems nominal",
            ))
            self.state = "NORMAL"
        elif msg_type == 0x10:  # DATA_FRAME (TASK_REQ for us)
            decoded = decode_data_frame(payload) or {}
            if decoded.get("type") == "TASK_REQ":
                task_id = decoded.get("task_id", "?")
                self.write_to_dock(encode_task_ack_json(
                    self.module_id, task_id, success=True,
                ))
                self.write_to_dock(encode_task_start_json(
                    self.module_id, task_id, time.time(),
                ))
                self.current_task_id = task_id
                self.state = "FIELD_OPS"
                self.in_field.set()


# ---------------------------------------------------------------------------
# Dock orchestrator
# ---------------------------------------------------------------------------

def make_pty_pair() -> Tuple[int, int]:
    """Open a PTY pair in raw mode (so binary protocol bytes survive)."""
    master_fd, slave_fd = os.openpty()
    for fd in (master_fd, slave_fd):
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(
            termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
            | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
        )
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.CSIZE | termios.PARENB)
        attrs[2] |= termios.CS8
        attrs[3] &= ~(
            termios.ECHO | termios.ECHONL | termios.ICANON
            | termios.ISIG | termios.IEXTEN
        )
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return master_fd, slave_fd


def build_swarm() -> List[FakeRover]:
    """Spawn the 8-rover heterogeneous swarm with canonical witch names."""
    rovers: List[FakeRover] = []
    used: set = set()
    for payload, caps, count in SWARM_COMPOSITION:
        for _ in range(count):
            # Pick a fresh witch name — keep going until we get an unused one.
            name = get_witch_name()
            while name in used:
                name = get_witch_name()
            used.add(name)
            master_fd, slave_fd = make_pty_pair()
            rover_info = RoverInfo(
                module_id=name,
                status=RoverStatus.IDLE,
                payload=payload,
                capabilities=caps,
                battery_pct=85.0,
                position=(0.0, 0.0),
                heading=0.0,
            )
            rovers.append(FakeRover(
                module_id=name,
                payload=payload,
                capabilities=caps,
                master_fd=master_fd,
                slave_fd=slave_fd,
                info=rover_info,
            ))
    return rovers


def handshake_all(rovers: List[FakeRover], dock_id: str, coven_name: str) -> None:
    """Walk every rover through IDENTIFY → ACK → VERIFY_OK in parallel."""
    # IDENTIFY_REQUEST to all rovers simultaneously.
    for r in rovers:
        os.write(r.master_fd, encode_identify_request(
            dock_id=dock_id, coven_name=coven_name, assigned_name=r.module_id,
        ))
    # Drain all IDENTIFY_REPLYs (each rover's thread will respond async).
    deadline = time.time() + 3.0
    while time.time() < deadline:
        all_replied = all(r.state in ("IDENTIFY", "WAIT_VERIFY", "NORMAL")
                          for r in rovers)
        if all_replied:
            break
        time.sleep(0.05)
    # Send IDENTIFY_ACK + VERIFY_OK to each.
    for r in rovers:
        os.write(r.master_fd, encode_identify_ack(
            dock_id=dock_id, assigned_name=r.module_id,
            message=f"Welcome to {coven_name}",
        ))
    time.sleep(0.3)
    for r in rovers:
        os.write(r.master_fd, encode_verify_ok(
            dock_id=dock_id, module_id=r.module_id,
        ))
    # Wait for all to settle in NORMAL.
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if all(r.state == "NORMAL" for r in rovers):
            break
        time.sleep(0.05)


def render_swarm_table(rovers: List[FakeRover]) -> None:
    """Print a roster of the swarm — names, payloads, capabilities, state."""
    print(f"{BOLD}Swarm roster:{RESET}")
    for r in rovers:
        state_color = (ok(r.state) if r.state == "NORMAL"
                       else warn(r.state) if r.state in ("FIELD_OPS",)
                       else f"{DIM}{r.state}{RESET}")
        print(
            f"  {witch(r.module_id):>40}  "
            f"{DIM}payload={RESET}{payload_label(r.payload)}  "
            f"{DIM}caps=0x{r.capabilities:02x}  state={RESET}{state_color}"
        )


def run_auction(
    rovers: List[FakeRover],
    task_type: TaskType,
    target: Tuple[float, float],
    dock_id: str,
) -> Optional[FakeRover]:
    """Announce a task, collect bids from every rover, dispatch to winner."""
    banner(
        f"Auction: {task_type.value.upper()} → ({target[0]:.1f}, {target[1]:.1f})",
        CYAN,
    )

    mission = Mission(
        mission_id=f"{task_type.value}-{int(time.time())}",
        task_type=task_type,
        waypoints=[target],
        dock_return=(0.0, 0.0),
    )

    # Collect bids — every rover bids; print every one for the screenshot.
    bids: List[Tuple[FakeRover, int]] = []
    for r in rovers:
        bid = r.info.calculate_bid(mission, dock_position=(0.0, 0.0))
        bids.append((r, bid))

    # Render the bid sheet.
    print(f"{BOLD}Bids:{RESET}")
    capable = [(r, b) for r, b in bids if b < 999999]
    if capable:
        finite_min = min(b for _, b in capable)
    else:
        finite_min = None

    # Sort by bid (incompatibles last). Stable sort preserves swarm order.
    bids_sorted = sorted(bids, key=lambda x: (x[1] >= 999999, x[1]))
    for r, b in bids_sorted:
        is_winner = (finite_min is not None
                     and b == finite_min
                     and r is capable[0][0])
        if b >= 999999:
            tag = bad("excluded — incompatible")
            bid_str = bad(f"{b:>6}")
        elif is_winner:
            tag = ok("WINNER")
            bid_str = ok(f"{b:>6}")
        else:
            tag = f"{DIM}eligible{RESET}"
            bid_str = warn(f"{b:>6}")
        print(
            f"  {witch(r.module_id):>40}  "
            f"{DIM}payload={RESET}{payload_label(r.payload)}  "
            f"bid {bid_str}  {tag}"
        )

    if not capable:
        print(bad("\n  No rover is capable of this task. Skipping dispatch."))
        return None

    winner = capable[0][0]
    print(f"\n{ok(f'⚡ TASK_REQ → {pretty(winner.module_id)}')}  "
          f"{DIM}(lowest finite bid; first registered breaks ties){RESET}")

    # Send the task to the winner only. Other rovers stay docked.
    task = {
        "dock_id": dock_id,
        "module_id": winner.module_id,
        "task_id": mission.mission_id,
        "task": task_type.value,
        "waypoints": [{"x": target[0], "y": target[1],
                       "yaw": 0.0, "tolerance": 0.3}],
        "dock_x": 0.0,
        "dock_y": 0.0,
        "coverage_threshold": 0.8,
        "timeout": 60.0,
    }
    os.write(winner.master_fd, encode_task_request(task))
    return winner


def watch_winner_return(
    winner: FakeRover,
    rovers: List[FakeRover],
    timeout: float = 10.0,
) -> bool:
    """Wait for the winner to come back from FIELD_OPS, drain its inbox."""
    print(f"\n{warn(f'[STATE] {pretty(winner.module_id)} now FIELD_OPS — silent until return.')}")
    print(f"        {DIM}Other rovers continue heartbeating while docked.{RESET}\n")

    # Drain the winner's TASK_ACK / TASK_START which arrived before going silent.
    deadline = time.time() + 1.0
    while time.time() < deadline:
        for msg_type, payload in [(0, b"")] if False else []:
            pass
        try:
            r, _, _ = select.select([winner.master_fd], [], [], 0.05)
            if r:
                data = os.read(winner.master_fd, 4096)
                # Just consume; we don't need to display them.
                if data:
                    pass
        except OSError:
            break

    # Wait for the in_field flag to clear (rover's thread sets field_complete).
    completed = winner.field_complete.wait(timeout=timeout)
    if completed:
        # Drain the TASK_COMPLETE frame from the master side.
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                r, _, _ = select.select([winner.master_fd], [], [], 0.05)
                if not r:
                    break
                data = os.read(winner.master_fd, 4096)
                if not data:
                    break
            except OSError:
                break
        print(ok(
            f"[RETURNED] {pretty(winner.module_id)} re-docked, reported "
            f"TASK_COMPLETE after {winner.field_duration:.1f} s. "
            f"Back in NORMAL."
        ))
        winner.field_complete.clear()
        return True
    else:
        print(bad(
            f"[TIMEOUT] {pretty(winner.module_id)} did not return within "
            f"{timeout:.0f} s."
        ))
        return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    runtime = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

    banner("COVEN 8-Rover Swarm Demo", CYAN)
    coven_name = get_coven_name()
    dock_id = f"dock-{coven_name.lower().replace('the_', '')}"
    print(f"{BOLD}Coven:{RESET} {coven(coven_name)}  "
          f"{DIM}(dock_id={dock_id}){RESET}\n")

    rovers = build_swarm()
    print(f"{BOLD}Spawning {len(rovers)} mock rovers as threads…{RESET}")
    threads = []
    for r in rovers:
        t = threading.Thread(target=r.run, daemon=True)
        t.start()
        threads.append(t)
    time.sleep(0.5)

    banner("Phase 1 — Handshake all 8 rovers", PASS)
    handshake_all(rovers, dock_id, coven_name)
    render_swarm_table(rovers)

    # Phase 2 — three auctions in succession.
    banner("Phase 2 — Auctions across the heterogeneous swarm", CYAN)
    auctions = [
        (TaskType.EXPLORE,    (5.0, 0.0)),
        (TaskType.SPECTRAL,   (3.0, 2.0)),
        (TaskType.BAROMETRIC, (0.0, 8.0)),
    ]

    deadline = time.time() + runtime
    for task_type, target in auctions:
        if time.time() > deadline:
            break
        winner = run_auction(rovers, task_type, target, dock_id)
        if winner is not None:
            watch_winner_return(winner, rovers, timeout=10.0)

    banner("Demo complete", PASS)
    print(f"{DIM}Roster after all auctions:{RESET}\n")
    render_swarm_table(rovers)

    # Tear down rover threads.
    for r in rovers:
        r._stop.set()
    time.sleep(0.2)
    for r in rovers:
        try:
            os.close(r.master_fd)
            os.close(r.slave_fd)
        except OSError:
            pass

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print(f"\n{WARN}Interrupted.{RESET}")
        sys.exit(0)
