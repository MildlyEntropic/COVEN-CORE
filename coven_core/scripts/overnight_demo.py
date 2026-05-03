#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
overnight_demo.py — leave-it-running heterogeneous swarm demo.

Spawns a persistent 8-rover swarm and a mission generator that issues
randomly-typed tasks at random intervals. Each mission goes through:

    auction → bid sheet → dispatch to capability-appropriate winner
           → rover IN_FIELD silent → return → TASK_COMPLETE → summary

Rover behavior is varied to exercise the dock's failure handling:

    ~80% nominal       — rover returns within expected window
    ~15% late          — rover returns past expected, before LOST deadline
                         (dock displays OVERDUE then RETURNED)
     ~5% lost          — rover never returns; dock retires the witch name
                         after 250% timeout, spawns a replacement with a
                         fresh witch name (same payload type)

The swarm stays at 8 rovers throughout: replacements come from the
canonical pool in coven_core.naming.

On Ctrl+C (or after the optional --hours runtime), the demo prints a
summary that includes the central economic argument: this swarm flew K
mission types over N hours; bespoke per-mission rovers cost ~$2.5B
each and address one mission profile each; COVEN-class infrastructure
addresses K profiles with one investment.

Usage:
    coven-overnight-demo                   # run until Ctrl+C
    coven-overnight-demo --hours 8         # run 8 hours then summarize
    coven-overnight-demo --interval 20     # mission every ~20s (default 30s)

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import argparse
import os
import random
import select
import signal
import sys
import termios
import threading
import time
import tty
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO))

from coven_core.frame_codec import (  # noqa: E402
    FrameParser,
    decode_data_frame,
    encode_identify_ack,
    encode_identify_request,
    encode_task_request,
    encode_verify_ok,
)
from coven_core.naming import (  # noqa: E402
    get_coven_name,
    get_witch_name,
    release_witch_name,
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


# --- Color palette (Flat UI, same as the other demos for cohesion) -------

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
PASS = "\033[38;2;46;204;113m"
FAIL = "\033[38;2;231;76;60m"
WARN = "\033[38;2;243;156;18m"
COVEN_COLOR = "\033[38;2;41;128;185m"
WITCH_COLOR = "\033[38;2;155;89;182m"
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


# --- Capability bit layout & swarm composition ---------------------------
#
# u8 bitmask, 7 of 8 bits assigned. The "drill" bit is shared between
# DRILL and EXCAVATOR — both are digging-class payloads; the PayloadType
# enum disambiguates. Aerial drones reuse the camera bit; HAULER gets
# the previously-free 0x80 cargo-bay bit.
CAP_ENCODERS     = 0x01
CAP_LIDAR        = 0x02
CAP_ULTRASONIC   = 0x04
CAP_CAMERA       = 0x08
CAP_SPECTROMETER = 0x10
CAP_DRILL        = 0x20  # also set on excavators (digging class)
CAP_BAROMETER    = 0x40
CAP_CARGO_BAY    = 0x80

# (PayloadType, capability bitmask, count) — exactly the user's request:
# 2 LiDAR + 2 Spectrometer + 2 Barometer + 1 Excavator + 1 Hauler + 1 Drone.
SWARM_COMPOSITION = [
    (PayloadType.LIDAR,         CAP_ENCODERS | CAP_LIDAR,                                2),
    (PayloadType.SPECTROMETER,  CAP_ENCODERS | CAP_LIDAR | CAP_SPECTROMETER,             2),
    (PayloadType.BAROMETER,     CAP_ENCODERS | CAP_BAROMETER,                            2),
    (PayloadType.EXCAVATOR,     CAP_ENCODERS | CAP_LIDAR | CAP_DRILL,                    1),
    (PayloadType.HAULER,        CAP_ENCODERS | CAP_LIDAR | CAP_CARGO_BAY,                1),
    (PayloadType.OPTICAL_DRONE, CAP_ENCODERS | CAP_CAMERA,                               1),
]


def payload_label(p: PayloadType) -> str:
    return {
        PayloadType.LIDAR:         "LiDAR        ",
        PayloadType.SPECTROMETER:  "Spectrometer ",
        PayloadType.DRILL:         "Drill        ",
        PayloadType.BAROMETER:     "Barometer    ",
        PayloadType.CAMERA:        "Camera       ",
        PayloadType.CARGO:         "Cargo        ",
        PayloadType.EXCAVATOR:     "Excavator    ",
        PayloadType.HAULER:        "Hauler       ",
        PayloadType.OPTICAL_DRONE: "Optical Drone",
    }.get(p, p.value)


# --- Physics & battery model --------------------------------------------
#
# Each payload type has physical properties (speed, battery capacity,
# drain rate, recharge rate, on-site overhead factor). Each task type
# has an at-site work time. A mission's predicted duration is then:
#
#     predicted_seconds = (2 × distance_to_waypoint) / rover.speed
#                       + task.work_time × rover.overhead_factor
#
# The actual duration is then a clustered random multiplier of the
# predicted: 95% of the time within 1% above to 5% above predicted
# (rovers slightly over-perform their estimate or run nominally),
# 4% with moderate variance (90–150% of predicted), and 1% in the
# heavy tail (150–300%). The 250% band crosses the LOST deadline,
# so the heavy tail naturally triggers the graceful-failure path
# the dock was built to handle. The user gets to watch this happen
# without anything being scripted.
#
# Battery: every rover starts at 100%. During FIELD_OPS the rover
# drains at `drain_rate_per_min` percent/minute. Docked, it
# recharges at `recharge_rate_per_min`. Any rover below 15% battery
# refuses to bid (sits docked recharging). This is the "witch trying
# to stay above 15% battery" rule.

@dataclass(frozen=True)
class RoverPhysics:
    max_speed: float                 # m/s
    drain_rate_per_min: float        # %battery / min during field ops
    recharge_rate_per_min: float     # %battery / min while docked
    overhead_factor: float           # multiplier on task work-time
    label: str                       # short label for summary


# Per-payload physics. Numbers chosen to give a recognizable feel:
# drones are fast but battery-poor; treaded vehicles are slow but
# capacious; excavators stay on-site forever.
#
# Battery dynamics (drain/recharge) are compressed for demo timescale:
# the 8-hour run is meant to represent days/weeks of real swarm
# operation, so battery cycles need to fit inside the run. Drain rates
# are tuned so that an actively dispatched rover hits the 30% bid
# penalty after roughly 60--90 minutes of sustained activity, falls
# below the 15% refuse threshold shortly after, then recharges back
# during a forced rest period — yielding multiple visible
# active/rest cycles per rover over an 8-hour run, including
# load-balanced rotation across redundant pairs.
ROVER_PHYSICS: Dict[PayloadType, RoverPhysics] = {
    PayloadType.LIDAR:        RoverPhysics(0.50, 1.00, 1.0, 1.00, "wheeled, light"),
    PayloadType.SPECTROMETER: RoverPhysics(0.45, 1.20, 1.0, 1.20, "wheeled, instrument-laden"),
    PayloadType.CAMERA:       RoverPhysics(0.55, 1.00, 1.0, 1.00, "wheeled, light"),
    PayloadType.BAROMETER:    RoverPhysics(0.40, 0.80, 1.0, 0.80, "wheeled, dead-reckoning"),
    PayloadType.DRILL:        RoverPhysics(0.30, 1.00, 0.8, 1.50, "wheeled, drill payload"),
    PayloadType.CARGO:        RoverPhysics(0.35, 0.70, 0.8, 1.00, "wheeled, cargo bay"),
    PayloadType.EXCAVATOR:    RoverPhysics(0.20, 0.70, 0.7, 2.00, "tracked, excavation arm"),
    PayloadType.HAULER:       RoverPhysics(0.30, 0.60, 0.7, 1.00, "treaded, heavy hauler"),
    PayloadType.OPTICAL_DRONE: RoverPhysics(8.0, 6.00, 2.0, 0.50, "aerial 4-DOF, small battery"),
}


# Per-task on-site work time in seconds (before the rover's overhead
# factor is applied). EXPLORE is mostly transit + brief sampling;
# EXCAVATE is mostly on-site work; AERIAL_SURVEY is a quick overflight.
TASK_WORK_TIME: Dict[TaskType, float] = {
    TaskType.EXPLORE:       30.0,
    TaskType.SURVEY:        60.0,
    TaskType.SPECTRAL:      180.0,
    TaskType.BAROMETRIC:    120.0,
    TaskType.SAMPLE:        600.0,
    TaskType.DELIVER:       60.0,
    TaskType.EXCAVATE:      900.0,
    TaskType.HAUL:          300.0,
    TaskType.AERIAL_SURVEY: 30.0,
}

# Battery refusal threshold — rovers below this skip auctions and
# stay docked recharging.
BATTERY_REFUSE_BELOW = 15.0  # %


def predicted_duration(
    rover_payload: PayloadType,
    task_type: TaskType,
    target: Tuple[float, float],
) -> float:
    """Physics-derived predicted mission duration in seconds.
    `predicted = round_trip / speed + task_work × overhead`."""
    physics = ROVER_PHYSICS[rover_payload]
    distance_to_target = (target[0] ** 2 + target[1] ** 2) ** 0.5
    round_trip = 2.0 * distance_to_target
    travel = round_trip / physics.max_speed
    work = TASK_WORK_TIME.get(task_type, 60.0) * physics.overhead_factor
    return travel + work


def roll_actual_factor() -> float:
    """Sample a mission's actual-vs-predicted duration multiplier.

    The distribution is calibrated so an 8-hour overnight run at default
    cadence (~320 dispatches) typically sees zero or one catastrophic
    loss, not the half-dozen the original 1% heavy-tail rate produced.
    A 9-rover swarm isn't truly disposable at demo cadence — losing
    multiple rovers per night looks like routine failure, not graceful
    exception handling.

        90.0%   factor in [0.99, 1.05]   — nominal completion
         9.0%   factor in [0.90, 1.50]   — moderate variance, returns ON TIME or LATE
         0.9%   factor in [1.50, 2.50]   — LATE tail; exceeds expected,
                                           absorbed within the dock's 250%
                                           LOST deadline → OVERDUE then RETURNED
         0.1%   factor in [2.50, 5.00]   — catastrophic; exceeds the LOST
                                           deadline → witch name retired,
                                           replacement rover spawned

    Expected LOST events per overnight run ≈ 0.32 (about one in three
    nights). Most runs end with zero rovers retired. Occasional nights
    show the full graceful-failure path firing organically. The
    single-rover demo (`coven-demo lose`) still exists for guaranteed
    LOST-flow screenshots.
    """
    r = random.random()
    if r < 0.90:
        return random.uniform(0.99, 1.05)
    elif r < 0.99:
        return random.choice([
            random.uniform(0.90, 0.99),
            random.uniform(1.05, 1.50),
        ])
    elif r < 0.999:
        return random.uniform(1.50, 2.50)
    else:
        return random.uniform(2.50, 5.00)


# --- FakeRover -----------------------------------------------------------

@dataclass
class FakeRover:
    """One simulated rover. Owns a PTY pair end and runs a thread."""
    module_id: str
    payload: PayloadType
    capabilities: int
    master_fd: int
    slave_fd: int
    info: RoverInfo
    behavior: str = "nominal"  # nominal | late | lost (set per mission)
    state: str = "BOOT"
    parser: FrameParser = field(default_factory=FrameParser)
    in_field: threading.Event = field(default_factory=threading.Event)
    field_complete: threading.Event = field(default_factory=threading.Event)
    current_task_id: Optional[str] = None
    current_task_type: Optional[TaskType] = None
    field_started_at: Optional[float] = None
    nominal_field_duration: float = 0.0  # physics prediction for the current mission
    actual_field_duration: float = 0.0   # what the rover actually takes
    missions_completed: int = 0
    missions_lost: int = 0
    is_busy: bool = False                # True while the rover is on a mission
    # Battery state: percent, 0–100. Updated continuously while docked
    # (recharge) and snapped on field-ops completion (drain).
    battery_pct: float = 100.0
    last_battery_tick: float = field(default_factory=time.time)
    _stop: threading.Event = field(default_factory=threading.Event)

    @property
    def physics(self) -> RoverPhysics:
        return ROVER_PHYSICS[self.payload]

    def tick_recharge(self) -> None:
        """Recharge while docked. Capped at 100%. Called on the rover's
        own thread tick so it integrates over real wall-clock time."""
        if self.is_busy:
            return
        now = time.time()
        elapsed_min = (now - self.last_battery_tick) / 60.0
        self.battery_pct = min(
            100.0,
            self.battery_pct + elapsed_min * self.physics.recharge_rate_per_min,
        )
        self.last_battery_tick = now
        # Mirror the figure into RoverInfo so the auctioneer's bid math
        # picks it up (calculate_bid penalizes <30% battery, bonuses >80%).
        self.info.battery_pct = self.battery_pct

    def drain_battery(self, field_seconds: float) -> None:
        """Apply mission-time battery drain at this rover's drain rate."""
        drain = (field_seconds / 60.0) * self.physics.drain_rate_per_min
        self.battery_pct = max(0.0, self.battery_pct - drain)
        self.info.battery_pct = self.battery_pct
        self.last_battery_tick = time.time()

    def write_to_dock(self, frame: bytes) -> None:
        try:
            os.write(self.slave_fd, frame)
        except OSError:
            pass

    def read_from_dock(self, timeout: float = 0.1) -> List[Tuple[int, bytes]]:
        r, _, _ = select.select([self.slave_fd], [], [], timeout)
        if not r:
            return []
        try:
            data = os.read(self.slave_fd, 4096)
        except OSError:
            return []
        return list(self.parser.feed(data))

    def run(self) -> None:
        last_heartbeat = 0.0
        while not self._stop.is_set():
            now = time.time()

            if self.in_field.is_set():
                if self.field_started_at is None:
                    self.field_started_at = now
                target_duration = self.actual_field_duration
                # behavior == "lost" sets actual_field_duration to inf, so
                # this branch never triggers for a lost rover — the dock's
                # 250% LOST deadline catches it.
                if now - self.field_started_at >= target_duration:
                    elapsed = now - self.field_started_at
                    self.drain_battery(elapsed)
                    self.write_to_dock(encode_task_complete_json(
                        module_id=self.module_id,
                        task_id=self.current_task_id or "?",
                        success=True,
                        coverage=1.0,
                        duration=elapsed,
                    ))
                    self.in_field.clear()
                    self.field_started_at = None
                    self.current_task_id = None
                    self.state = "NORMAL"
                    self.missions_completed += 1
                    self.field_complete.set()
                time.sleep(0.05)
                continue

            # Docked: recharge battery and emit heartbeats at 1 Hz.
            self.tick_recharge()

            if self.state in ("WAIT_VERIFY", "NORMAL") and now - last_heartbeat >= 1.0:
                status = "IDLE" if self.state == "NORMAL" else "STARTUP"
                self.write_to_dock(encode_heartbeat(
                    module_id=self.module_id,
                    battery_pct=self.battery_pct,
                    mission_status=status,
                    x=self.info.position[0],
                    y=self.info.position[1],
                    theta=self.info.heading,
                ))
                last_heartbeat = now

            for msg_type, payload in self.read_from_dock(timeout=0.1):
                self._handle_dock_message(msg_type, payload)

    def _handle_dock_message(self, msg_type: int, payload: bytes) -> None:
        if msg_type == 0x01:
            self.write_to_dock(encode_identify_reply(
                module_id=self.module_id,
                module_type="ReconRover",
                firmware="0.1.0",
                battery_pct=self.info.battery_pct,
                status="OK",
                capabilities=self.capabilities,
            ))
            self.state = "IDENTIFY"
        elif msg_type == 0x05:
            self.state = "WAIT_VERIFY"
        elif msg_type == 0x03:
            self.write_to_dock(encode_verify_rep(
                module_id=self.module_id, success=True,
                failed_checks=[], note="All systems nominal",
            ))
            self.state = "NORMAL"
        elif msg_type == 0x10:
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


def make_pty_pair() -> Tuple[int, int]:
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


# --- Stats accumulator ---------------------------------------------------

@dataclass
class SwarmStats:
    started_at: float = field(default_factory=time.time)
    missions_dispatched: int = 0
    missions_completed: int = 0
    missions_late: int = 0
    missions_lost: int = 0
    by_task_type: Counter = field(default_factory=Counter)
    by_winner_payload: Counter = field(default_factory=Counter)
    field_durations: List[float] = field(default_factory=list)
    rovers_retired: List[Tuple[str, str]] = field(default_factory=list)
    rovers_replaced: List[Tuple[str, str]] = field(default_factory=list)
    per_rover_completed: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    per_rover_lost: Dict[str, int] = field(default_factory=lambda: defaultdict(int))

    def elapsed(self) -> float:
        return time.time() - self.started_at

    def rate_per_hour(self) -> float:
        e = self.elapsed()
        return self.missions_dispatched / (e / 3600.0) if e > 0 else 0.0


# --- Dock orchestrator ---------------------------------------------------

def build_swarm() -> List[FakeRover]:
    rovers: List[FakeRover] = []
    used: set = set()
    for payload, caps, count in SWARM_COMPOSITION:
        for _ in range(count):
            name = get_witch_name()
            while name in used:
                name = get_witch_name()
            used.add(name)
            rovers.append(_make_rover(name, payload, caps))
    return rovers


def _make_rover(name: str, payload: PayloadType, caps: int) -> FakeRover:
    master_fd, slave_fd = make_pty_pair()
    info = RoverInfo(
        module_id=name,
        status=RoverStatus.IDLE,
        payload=payload,
        capabilities=caps,
        battery_pct=85.0,
        position=(0.0, 0.0),
        heading=0.0,
    )
    return FakeRover(
        module_id=name, payload=payload, capabilities=caps,
        master_fd=master_fd, slave_fd=slave_fd, info=info,
    )


def handshake(rover: FakeRover, dock_id: str, coven_name: str) -> None:
    os.write(rover.master_fd, encode_identify_request(
        dock_id=dock_id, coven_name=coven_name, assigned_name=rover.module_id,
    ))
    deadline = time.time() + 3.0
    while time.time() < deadline and rover.state == "BOOT":
        time.sleep(0.05)
    os.write(rover.master_fd, encode_identify_ack(
        dock_id=dock_id, assigned_name=rover.module_id,
        message=f"Welcome to {coven_name}",
    ))
    time.sleep(0.2)
    os.write(rover.master_fd, encode_verify_ok(
        dock_id=dock_id, module_id=rover.module_id,
    ))
    deadline = time.time() + 3.0
    while time.time() < deadline and rover.state != "NORMAL":
        time.sleep(0.05)


def replace_rover(
    old: FakeRover,
    rovers: List[FakeRover],
    threads: List[threading.Thread],
    dock_id: str,
    coven_name: str,
    stats: SwarmStats,
) -> FakeRover:
    """A rover was declared LOST. Retire its name; spawn a replacement
    with the same payload type but a fresh witch name from the canonical
    pool. Returns the new rover (and adds it to `rovers`)."""
    release_witch_name(old.module_id)
    used = {r.module_id for r in rovers if r is not old}
    new_name = get_witch_name()
    while new_name in used:
        new_name = get_witch_name()
    new_rover = _make_rover(new_name, old.payload, old.capabilities)
    # Stop the old thread and clean up.
    old._stop.set()
    try:
        os.close(old.master_fd)
        os.close(old.slave_fd)
    except OSError:
        pass
    # Splice in.
    idx = rovers.index(old)
    rovers[idx] = new_rover
    t = threading.Thread(target=new_rover.run, daemon=True)
    t.start()
    threads[idx] = t
    handshake(new_rover, dock_id, coven_name)
    stats.rovers_retired.append((old.module_id, old.payload.value))
    stats.rovers_replaced.append((new_rover.module_id, new_rover.payload.value))
    print(ok(
        f"  [REPLACE] {pretty(old.module_id)} retired → "
        f"{pretty(new_rover.module_id)} ({payload_label(new_rover.payload).strip()}) "
        f"joined the coven."
    ))
    return new_rover


_LAST_TASK_TYPE: Optional[TaskType] = None


def random_mission_profile() -> Tuple[TaskType, Tuple[float, float]]:
    """Pick a random task type + waypoint for the next mission.

    The pool spans every task type the swarm can address — including the
    three added with the expanded composition (excavate, haul, aerial
    survey). Sampling is *biased against repeating the immediately
    previous task type*: the previous type gets a 0.25× weight relative
    to the others. This produces a more varied stream of dispatches
    over short windows (no long runs of consecutive same-type missions)
    while preserving roughly uniform coverage of the type histogram
    across a long run.
    """
    global _LAST_TASK_TYPE
    task_types = [
        TaskType.EXPLORE,
        TaskType.SPECTRAL,
        TaskType.SAMPLE,
        TaskType.SURVEY,
        TaskType.BAROMETRIC,
        TaskType.EXCAVATE,
        TaskType.HAUL,
        TaskType.AERIAL_SURVEY,
    ]
    weights = [0.25 if t == _LAST_TASK_TYPE else 1.0 for t in task_types]
    t = random.choices(task_types, weights=weights, k=1)[0]
    _LAST_TASK_TYPE = t
    radius = random.uniform(2.0, 12.0)
    angle = random.uniform(0, 2 * 3.14159)
    return t, (radius * cos_(angle), radius * sin_(angle))


def cos_(x: float) -> float:
    import math
    return math.cos(x)


def sin_(x: float) -> float:
    import math
    return math.sin(x)


_swarm_lock = threading.Lock()  # serializes rover-list mutation across threads


def _watch_mission(
    winner: FakeRover,
    mission_id: str,
    task_type: TaskType,
    behavior: str,
    rovers: List[FakeRover],
    threads: List[threading.Thread],
    dock_id: str,
    coven_name: str,
    stats: SwarmStats,
) -> None:
    """Run in its own thread for the lifetime of one mission.

    The main loop dispatches a winner and moves on; this watcher waits
    for the rover to return (or time out as LOST) and prints the result
    asynchronously, without blocking subsequent dispatches. That's what
    makes concurrent rovers in field possible — multiple watchers can
    be active simultaneously, each tracking its own winner.
    """
    lost_deadline = winner.nominal_field_duration * 2.5
    completed = winner.field_complete.wait(timeout=lost_deadline + 1.0)

    # Drain any final bytes from the master end (TASK_COMPLETE etc.).
    deadline = time.time() + 0.5
    while time.time() < deadline:
        try:
            r, _, _ = select.select([winner.master_fd], [], [], 0.05)
            if not r:
                break
            os.read(winner.master_fd, 4096)
        except OSError:
            break

    with _swarm_lock:
        if completed:
            winner.field_complete.clear()
            is_late = (behavior == "late")
            if is_late:
                stats.missions_late += 1
            stats.missions_completed += 1
            stats.field_durations.append(winner.actual_field_duration)
            stats.per_rover_completed[winner.module_id] += 1
            tag = warn("LATE") if is_late else ok("ON TIME")
            print(
                f"  {tag} {witch(winner.module_id)} {DIM}({task_type.value}){RESET} "
                f"returned after {winner.actual_field_duration/60:.1f} min "
                f"{DIM}(expected {winner.nominal_field_duration/60:.1f}, "
                f"task {mission_id}){RESET}"
            )
            winner.is_busy = False
        else:
            stats.missions_lost += 1
            winner.missions_lost += 1
            stats.per_rover_lost[winner.module_id] += 1
            print(bad(
                f"  [LOST] {pretty(winner.module_id)} {DIM}({task_type.value}){RESET} "
                f"did not return within {lost_deadline/60:.1f} min. "
                f"Witch name retired."
            ))
            replace_rover(winner, rovers, threads, dock_id, coven_name, stats)


def run_one_mission(
    rovers: List[FakeRover],
    threads: List[threading.Thread],
    dock_id: str,
    coven_name: str,
    stats: SwarmStats,
    sequence: int,
) -> None:
    """Generate a random mission, auction it among *free* rovers, dispatch
    the winner, and hand off to a watcher thread. Returns immediately so
    the main loop can keep generating missions while rovers are in field.

    If no capable rover is free, the mission is logged as deferred —
    a real auctioneer would queue it; the demo just notes and moves on.
    """
    task_type, target = random_mission_profile()
    print(f"\n{CYAN}{BOLD}─── Mission #{sequence}: "
          f"{task_type.value.upper()} → ({target[0]:+.1f}, {target[1]:+.1f}) "
          f"───{RESET}")

    mission = Mission(
        mission_id=f"{task_type.value}-{sequence:04d}",
        task_type=task_type,
        waypoints=[target],
        dock_return=(0.0, 0.0),
    )

    with _swarm_lock:
        # Sync each docked rover's recharged battery into RoverInfo so
        # calculate_bid sees the current charge level.
        for r in rovers:
            if not r.is_busy:
                r.tick_recharge()

        # Eligibility:
        #   1. Not currently in the field.
        #   2. Battery at or above the refusal threshold (rover wants to
        #      stay above 15% to avoid stranding itself; below this it
        #      sits docked recharging and lets someone else take the bid).
        free_rovers = [r for r in rovers if not r.is_busy]
        eligible = [r for r in free_rovers if r.battery_pct >= BATTERY_REFUSE_BELOW]
        battery_skipped = [r for r in free_rovers if r.battery_pct < BATTERY_REFUSE_BELOW]
        in_field_count = len(rovers) - len(free_rovers)

        bids = [(r, r.info.calculate_bid(mission, dock_position=(0.0, 0.0)))
                for r in eligible]
        capable = [(r, b) for r, b in bids if b < 999999]

        if not capable:
            # Possible reasons: no rover with this capability exists at
            # all (a configuration / spec error worth flagging), or all
            # capable rovers are temporarily unavailable (in field, or
            # below the battery refusal threshold). The latter case is
            # benign in a real auctioneer — it queues the mission and
            # dispatches when a capable rover becomes available — and
            # is not tracked as a metric here. We log briefly for
            # inspection and move on.
            all_capable_when_idle = [
                r for r in rovers
                if r.info.calculate_bid(mission, (0.0, 0.0)) < 999999
            ]
            if not all_capable_when_idle:
                print(bad(
                    f"  No rover in the swarm has the capability for "
                    f"{task_type.value}; skipping."
                ))
            else:
                print(DIM + "  No capable rover available right now "
                      "(in field or recharging); skipped." + RESET)
            return

        capable.sort(key=lambda x: x[1])
        winner = capable[0][0]
        winner_bid = capable[0][1]
        incompatible_count = len(bids) - len(capable)
        battery_note = (
            f", {len(battery_skipped)} skipped (low battery)"
            if battery_skipped else ""
        )
        print(
            f"  {DIM}{len(capable)} capable bidders, {incompatible_count} excluded"
            f"{battery_note}; "
            f"({in_field_count} of {len(rovers)} rovers already in field); "
            f"winner:{RESET} {witch(winner.module_id)} "
            f"{DIM}({payload_label(winner.payload).strip()}, "
            f"bid={winner_bid}, battery={winner.battery_pct:.0f}%){RESET}"
        )

        # Physics-derived predicted duration; clustered random factor for
        # the actual. The 1% heavy-tail roll exceeds 250% of predicted —
        # exactly the band the LOST timeout was built for.
        predicted = predicted_duration(winner.payload, task_type, target)
        factor = roll_actual_factor()
        actual = predicted * factor

        winner.current_task_type = task_type
        winner.nominal_field_duration = predicted
        winner.actual_field_duration = actual
        if factor > 2.5:
            behavior = "lost"   # will exceed dock's 250% deadline
            winner.actual_field_duration = float("inf")
        elif factor > 1.05:
            behavior = "late"
        else:
            behavior = "nominal"
        winner.behavior = behavior
        winner.is_busy = True

        stats.missions_dispatched += 1
        stats.by_task_type[task_type.value] += 1
        stats.by_winner_payload[winner.payload.value] += 1

        physics = winner.physics
        target_dist = (target[0]**2 + target[1]**2) ** 0.5
        print(
            f"  {DIM}physics: dist {target_dist:.1f} m, speed "
            f"{physics.max_speed:.2f} m/s ({physics.label}); "
            f"predicted {predicted/60:.1f} min, "
            f"actual {factor*100:.0f}% of predicted "
            f"→ {actual/60:.1f} min"
            + (f" ({warn('!')}{DIM} heavy-tail roll → expect LOST){RESET}"
               if factor > 2.5 else "")
            + RESET
        )

        # Dispatch: send TASK_REQ on the winner's UART. The rover thread
        # reads the request and transitions to FIELD_OPS internally.
        task = {
            "dock_id": dock_id, "module_id": winner.module_id,
            "task_id": mission.mission_id, "task": task_type.value,
            "waypoints": [{"x": target[0], "y": target[1],
                           "yaw": 0.0, "tolerance": 0.3}],
            "dock_x": 0.0, "dock_y": 0.0,
            "coverage_threshold": 0.8, "timeout": 60.0,
        }
        os.write(winner.master_fd, encode_task_request(task))

    # Drain TASK_ACK / TASK_START outside the lock so we don't block
    # other watchers.
    deadline = time.time() + 0.5
    while time.time() < deadline:
        try:
            r, _, _ = select.select([winner.master_fd], [], [], 0.05)
            if r:
                os.read(winner.master_fd, 4096)
        except OSError:
            break

    # Hand off to a watcher thread. Daemon=True so the thread doesn't
    # keep the process alive after summary printout.
    threading.Thread(
        target=_watch_mission,
        args=(winner, mission.mission_id, task_type, behavior,
              rovers, threads, dock_id, coven_name, stats),
        daemon=True,
    ).start()


# --- Summary report ------------------------------------------------------

def print_summary(stats: SwarmStats, rovers: List[FakeRover],
                  coven_name: str) -> None:
    elapsed = stats.elapsed()
    hours = elapsed / 3600.0
    banner("Overnight Swarm Demo — Final Summary", PASS)

    print(f"{BOLD}Coven:{RESET}        {coven(coven_name)}")
    print(f"{BOLD}Runtime:{RESET}      {elapsed/60:.1f} min ({hours:.2f} h)")
    print(f"{BOLD}Throughput:{RESET}   "
          f"{ok(f'{stats.rate_per_hour():.1f} missions/hour')}")
    print()

    print(f"{BOLD}Mission outcomes:{RESET}")
    print(f"  {ok('Completed on time:'):>40}  {stats.missions_completed - stats.missions_late}")
    print(f"  {warn('Completed late (within 250% margin):'):>50}  {stats.missions_late}")
    print(f"  {bad('Lost (rover retired):'):>40}  {stats.missions_lost}")
    print(f"  {DIM}{'Total dispatched:':>40}{RESET}  {stats.missions_dispatched}")
    if stats.field_durations:
        avg_min = sum(stats.field_durations) / len(stats.field_durations) / 60.0
        max_min = max(stats.field_durations) / 60.0
        min_min = min(stats.field_durations) / 60.0
        print(f"  {DIM}{'Field-time per mission:':>40}{RESET}  "
              f"avg {avg_min:.1f} min, range {min_min:.1f}–{max_min:.1f} min")
    print()

    if stats.by_task_type:
        print(f"{BOLD}Mission mix (by task type):{RESET}")
        for t, c in stats.by_task_type.most_common():
            bar = "█" * min(40, c)
            print(f"  {t.upper():>15}  {bar} {c}")
        print()

    if stats.by_winner_payload:
        print(f"{BOLD}Dispatched payload (winning rover type):{RESET}")
        for p, c in stats.by_winner_payload.most_common():
            bar = "█" * min(40, c)
            print(f"  {p.upper():>15}  {bar} {c}")
        print()

    print(f"{BOLD}Per-rover scoreboard (current swarm):{RESET}")
    in_field_now = []
    for r in rovers:
        comp = stats.per_rover_completed.get(r.module_id, 0)
        lost = stats.per_rover_lost.get(r.module_id, 0)
        busy_tag = (warn("[in field]") if r.is_busy else "")
        if r.is_busy:
            in_field_now.append(r)
        print(
            f"  {witch(r.module_id):>40}  "
            f"{DIM}{payload_label(r.payload)}{RESET}  "
            f"{ok(f'{comp:>3} completed')}  "
            + (bad(f"{lost} lost") if lost else f"{DIM}0 lost{RESET}")
            + ("  " + busy_tag if busy_tag else "")
        )
    if in_field_now:
        print()
        print(
            f"  {warn(f'{len(in_field_now)} rover(s) still in field at summary time')} "
            f"{DIM}— their missions did not get a chance to complete in this run.{RESET}"
        )
    print()

    if stats.rovers_retired:
        print(f"{BOLD}Rover turnover (retired → replaced):{RESET}")
        for (old_id, payload_v), (new_id, _) in zip(
                stats.rovers_retired, stats.rovers_replaced):
            print(
                f"  {witch(old_id):>40}  {DIM}({payload_v}){RESET}  "
                f"→  {witch(new_id)}"
            )
        print()

    # The economic argument the demo is designed to evidence.
    banner("What this swarm demonstrates", COVEN_COLOR)
    n_types = len(stats.by_task_type)
    print(
        f"  {BOLD}One swarm, {n_types} mission types served, "
        f"{stats.missions_dispatched} dispatches in "
        f"{hours:.2f} hours.{RESET}\n"
    )
    print(
        f"  {DIM}NASA's Decadal Survey identifies dozens of high-priority\n"
        f"  planetary science targets each cycle. Funding constraints mean\n"
        f"  ~5–10 of these get selected per decade and the remainder, however\n"
        f"  scientifically valid, never fly. The bespoke per-mission rover\n"
        f"  model concentrates total cost into single-purpose hardware: a\n"
        f"  $2.5B Curiosity flies one mission profile.{RESET}\n"
    )
    print(
        f"  {ok('COVEN inverts that economics.')} "
        f"{DIM}This swarm just executed{RESET}\n"
        f"  {DIM}{n_types} different mission profiles on the same dock running the\n"
        f"  same protocol code, dispatching to capability-appropriate rovers\n"
        f"  via the polymorphic auction. Adding a new mission type is a row\n"
        f"  in the payload-task matrix and a new rover specialization — not a\n"
        f"  new $2.5B procurement cycle.{RESET}\n"
    )
    if n_types > 0 and stats.missions_dispatched > 0:
        bespoke_per_mission = 2_500_000_000  # USD, Curiosity-class
        coven_per_dispatch_estimate = (
            bespoke_per_mission / stats.missions_dispatched
            if stats.missions_dispatched > 0 else 0
        )
        print(
            f"  {DIM}If COVEN-class infrastructure cost the same as a single\n"
            f"  bespoke mission, the per-dispatch cost in this run would be\n"
            f"  ${coven_per_dispatch_estimate/1_000_000:.1f}M — and the cost\n"
            f"  scales {ok('down')}{DIM} with every additional mission, not up.{RESET}\n"
        )

    print(
        f"{DIM}Every Decadal-priority study currently shelved for budget\n"
        f"reasons is a candidate this kind of infrastructure could fly\n"
        f"without a new procurement.{RESET}\n"
    )


# --- Main ----------------------------------------------------------------

_SHUTDOWN = threading.Event()


def _on_sigint(signum, frame):
    _SHUTDOWN.set()


def main() -> int:
    parser = argparse.ArgumentParser(prog="coven-overnight-demo")
    parser.add_argument("--hours", type=float, default=0.0,
                        help="Run for this many hours then summarize. "
                             "Default 0 = run until Ctrl+C.")
    parser.add_argument("--interval", type=float, default=30.0,
                        help="Seconds between mission generations. "
                             "Default 30 — chosen so the 9-rover swarm "
                             "approaches sustained utilization with most "
                             "rovers in field most of the time, given an "
                             "average mission duration of ~3-4 minutes. "
                             "Per-mission durations are physics-derived "
                             "(distance/speed + on-site work) so they "
                             "vary by rover-task pair from ~30s aerial "
                             "flights to ~30min excavator missions.")
    parser.add_argument("--seed", type=int, default=None,
                        help="RNG seed for reproducible runs.")
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)

    banner("COVEN Overnight Swarm Demo", CYAN)
    coven_name = get_coven_name()
    dock_id = f"dock-{coven_name.lower().replace('the_', '')}"
    print(f"{BOLD}Coven:{RESET}     {coven(coven_name)}  {DIM}(dock_id={dock_id}){RESET}")
    print(f"{BOLD}Cadence:{RESET}   one mission every ~{args.interval:.0f} s")
    if args.hours > 0:
        print(f"{BOLD}Runtime:{RESET}   {args.hours:.1f} h then summarize")
    else:
        print(f"{BOLD}Runtime:{RESET}   until Ctrl+C")
    print()

    rovers = build_swarm()
    print(f"{BOLD}Spawning {len(rovers)}-rover swarm…{RESET}")
    threads: List[threading.Thread] = []
    for r in rovers:
        t = threading.Thread(target=r.run, daemon=True)
        t.start()
        threads.append(t)
    time.sleep(0.5)

    print(f"{BOLD}Handshaking each rover…{RESET}")
    for r in rovers:
        handshake(r, dock_id, coven_name)
    print(ok(f"  {len(rovers)} rovers in NORMAL state, ready for tasks.\n"))

    stats = SwarmStats()
    deadline = time.time() + args.hours * 3600.0 if args.hours > 0 else None
    sequence = 0
    next_mission = time.time() + 1.0  # first mission after a 1s settle

    try:
        while not _SHUTDOWN.is_set():
            if deadline is not None and time.time() >= deadline:
                break
            now = time.time()
            if now >= next_mission:
                sequence += 1
                run_one_mission(rovers, threads, dock_id, coven_name,
                                stats, sequence)
                # Jitter the cadence ±25% so the demo doesn't tick like a
                # metronome — feels more like a real frontier dispatcher.
                jitter = random.uniform(0.75, 1.25)
                next_mission = time.time() + args.interval * jitter
            time.sleep(0.5)
    finally:
        print_summary(stats, rovers, coven_name)
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
    raise SystemExit(main())
