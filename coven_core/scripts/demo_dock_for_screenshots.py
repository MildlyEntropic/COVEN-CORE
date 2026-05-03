#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
demo_dock_for_screenshots.py — one-shot "fake dock" for thesis demos.

Creates a virtual UART (PTY pair), prints the rover-side device path so
you can point `coven-rover --mock` at it, and walks the rover through a
full COVEN handshake while logging every byte of protocol traffic to
stdout. Designed to produce clean screenshots of:

  * The IDENTIFY_REQUEST / IDENTIFY_REPLY / IDENTIFY_ACK exchange
  * The VERIFY_REQ / VERIFY_REP / VERIFY_OK exchange
  * Heartbeats arriving at 1 Hz
  * A TASK_REQ being dispatched and the rover transitioning to FIELD_OPS

This script needs neither ROS2 nor Gazebo nor Docker. It runs on any
machine with Python and the COVEN repo. The companion rover side runs
the actual production Rust firmware in `--mock` mode.

Usage (two terminals):

  Terminal 1 — start the demo dock:
      cd Code/COVEN-CORE/coven_core
      python3 scripts/demo_dock_for_screenshots.py
    The script prints the PTY slave path and the path to a generated
    config file the rover should use.

  Terminal 2 — start the rover in mock mode:
      cd Code/COVEN-CORE/coven_core/rover
      ./target/release/coven-rover --mock \
          --config /tmp/coven_demo_rover.toml \
          --verbose
    Or whatever the demo dock printed.

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import json
import os
import select
import struct
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Optional

# Make coven_core importable when run from the repo root.
_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO))

from coven_core.frame_codec import (  # noqa: E402
    FrameParser,
    decode_data_frame,
    decode_heartbeat,
    decode_identify_reply,
    encode_identify_ack,
    encode_identify_request,
    encode_task_request,
    encode_verify_fail,
    encode_verify_ok,
    SUBTYPE_VERIFY_REP,
)
from coven_core.naming import (  # noqa: E402
    COVEN_NAMES,
    WITCH_NAMES,
    get_coven_name,
    get_witch_name,
    release_witch_name,
)


# ANSI colors for readable terminal output (great for screenshots).
#
# The semantic palette is drawn from Flat UI Colors (flatuicolors.com),
# which is engineered for consistent saturation across hues — so the
# five accent colors (PASS green, FAIL red, WARN orange, COVEN blue,
# WITCH purple) carry equal visual weight and don't fight each other.
#
# Why these specific picks rather than pure CSS primaries:
#   * Pure #0000FF blue is eye-searing on most modern terminals and
#     reads as "link" / "error" rather than "institution".
#   * Pure #112358 navy is too dark to read against a dark terminal
#     background, which is where these screenshots will live.
#   * Picking from one curated palette gives the demo a cohesive look
#     in a screenshot rather than a Christmas-tree mix of saturations.
#
# Hue choices:
#   COVEN  Belize Hole #2980B9 — institutional, calm, authoritative
#   WITCH  Wisteria   #9B59B6 — magical, individual, thematically apt
#   PASS   Emerald    #2ECC71 — vivid green for OK / heartbeats
#   FAIL   Alizarin   #E74C3C — vivid red for rejection / failure
#   WARN   Orange     #F39C12 — saturated orange for soft failures

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"

# 8-color basic palette (used for structural elements like direction
# markers and step banners — kept distinct from the semantic accents).
BLUE_BASIC = "\033[34m"
YELLOW = "\033[33m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"

# Semantic accent palette (truecolor, Flat UI).
PASS = "\033[38;2;46;204;113m"        # #2ECC71 Emerald
FAIL = "\033[38;2;231;76;60m"         # #E74C3C Alizarin
WARN = "\033[38;2;243;156;18m"        # #F39C12 Orange
COVEN_COLOR = "\033[38;2;41;128;185m" # #2980B9 Belize Hole
WITCH_COLOR = "\033[38;2;155;89;182m" # #9B59B6 Wisteria

# Backwards-compat aliases used by the existing log_send / banner code.
GREEN = PASS
BLUE = BLUE_BASIC
RED = FAIL


def pretty(name: str) -> str:
    """Display form of a canonical name. Underscores in identifiers
    (`The_Hags_of_Dun_Broch`, `Morgan_Le_Fay`) become spaces in human
    output (`The Hags of Dun Broch`, `Morgan Le Fay`). The on-the-wire
    identifier is unchanged — only what gets printed is affected."""
    return name.replace("_", " ")


def coven(name: str) -> str:
    """Render a coven name in its dedicated #0000FF blue."""
    return f"{COVEN_COLOR}{BOLD}{pretty(name)}{RESET}"


def witch(name: str) -> str:
    """Render a witch name in its dedicated #112358 dark navy."""
    return f"{WITCH_COLOR}{BOLD}{pretty(name)}{RESET}"


def ok(text: str) -> str:
    """Mark something as PASS / OK in vivid green."""
    return f"{PASS}{BOLD}{text}{RESET}"


def bad(text: str) -> str:
    """Mark something as FAIL in vivid red."""
    return f"{FAIL}{BOLD}{text}{RESET}"


def warn(text: str) -> str:
    """Mark something as a warning in orange."""
    return f"{WARN}{BOLD}{text}{RESET}"


def banner(text: str, color: str = CYAN) -> None:
    line = "═" * (len(text) + 4)
    print(f"\n{color}{BOLD}{line}{RESET}")
    print(f"{color}{BOLD}║ {text} ║{RESET}")
    print(f"{color}{BOLD}{line}{RESET}\n")


def log_send(label: str, msg_type: int, frame: bytes) -> None:
    print(
        f"{BLUE}{BOLD}[DOCK→ROVER]{RESET} {GREEN}{label}{RESET} "
        f"{DIM}(type=0x{msg_type:02x}, {len(frame)} bytes on wire){RESET}"
    )


def log_recv(label: str, msg_type: int, payload: bytes) -> None:
    print(
        f"{MAGENTA}{BOLD}[ROVER→DOCK]{RESET} {YELLOW}{label}{RESET} "
        f"{DIM}(type=0x{msg_type:02x}, {len(payload)} bytes payload){RESET}"
    )


def write_demo_config(rover_pty_path: str) -> str:
    """Write a TOML config that points the rover at the PTY and disables
    real LiDAR. Returns the path to the config file."""
    config_path = "/tmp/coven_demo_rover.toml"
    with open(config_path, "w") as f:
        f.write(f"""# Auto-generated by demo_dock_for_screenshots.py
# Points the rover firmware at the demo dock's virtual UART.

rover_id = "Morgan_Le_Fay"
coven_name = "The_Graeae"

[dock_uart]
port = "{rover_pty_path}"
baud_rate = 115200

[hardware.motors]
front_left_pwm = 12
front_left_in1 = 5
front_left_in2 = 6
front_right_pwm = 13
front_right_in1 = 16
front_right_in2 = 26
standby_1 = 17
rear_left_pwm = 18
rear_left_in1 = 19
rear_left_in2 = 20
rear_right_pwm = 21
rear_right_in1 = 25
rear_right_in2 = 8
standby_2 = 7
pwm_frequency = 1000.0
wheel_base = 0.298
wheel_radius = 0.0325
max_rpm = 130.0

[hardware.encoders]
front_left_a = 23
front_left_b = 24
front_right_a = 27
front_right_b = 22
rear_left_a = 9
rear_left_b = 10
rear_right_a = 11
rear_right_b = 4

[hardware.lidar]
port = "/dev/null"
baud_rate = 460800

[hardware.battery]
i2c_bus = 1
i2c_address = 0x48

[timing]
control_rate = 20
""")
    return config_path


def main() -> int:
    banner("COVEN Demo Dock — Screenshot Mode", CYAN)

    # Pick proper names from the canonical lore (coven_core/naming.py).
    # The dock claims a coven; it assigns the rover a witch name via
    # the IDENTIFY_REQUEST so the screenshots show recognizable identities
    # rather than the rover's TOML default.
    coven_name = get_coven_name()
    rover_assigned_name = get_witch_name()
    print(f"{BOLD}Coven for this run:{RESET}  {coven(coven_name)}  "
          f"{DIM}(one of {len(COVEN_NAMES)} canonical covens){RESET}")
    print(f"{BOLD}Rover witch name:{RESET}     {witch(rover_assigned_name)}  "
          f"{DIM}(one of {len(WITCH_NAMES)} canonical witches){RESET}\n")

    # Open a PTY pair. The slave path is what the rover firmware connects to.
    master_fd, slave_fd = os.openpty()
    rover_pty_path = os.ttyname(slave_fd)
    print(f"{BOLD}PTY slave path (rover should connect here):{RESET} "
          f"{GREEN}{rover_pty_path}{RESET}\n")

    # CRITICAL: PTYs default to canonical/cooked terminal line discipline,
    # which mangles binary serial bytes (echoes input, eats 0x00, translates
    # CR/LF, etc.). For a binary COBS-framed protocol this destroys the
    # data. Put both ends into raw mode before any I/O happens.
    for fd in (master_fd, slave_fd):
        attrs = termios.tcgetattr(fd)
        # cfmakeraw() equivalent: turn off all line-discipline cooking.
        attrs[0] &= ~(  # iflag
            termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
            | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
        )
        attrs[1] &= ~termios.OPOST  # oflag — disable output processing
        attrs[2] &= ~(termios.CSIZE | termios.PARENB)
        attrs[2] |= termios.CS8
        attrs[3] &= ~(  # lflag
            termios.ECHO | termios.ECHONL | termios.ICANON
            | termios.ISIG | termios.IEXTEN
        )
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # Make a stable symlink so the screenshot recipe is the same every run.
    stable_link = "/tmp/coven_demo_uart"
    try:
        if os.path.lexists(stable_link):
            os.remove(stable_link)
        os.symlink(rover_pty_path, stable_link)
        print(f"{BOLD}Stable symlink:{RESET} {GREEN}{stable_link}{RESET}")
    except OSError as e:
        print(f"{YELLOW}WARNING: could not create symlink {stable_link}: {e}{RESET}")

    # Write a config the rover can use.
    config_path = write_demo_config(stable_link)
    print(f"{BOLD}Rover config written to:{RESET} {GREEN}{config_path}{RESET}\n")

    print(f"{BOLD}Now in another terminal, run:{RESET}")
    print(
        f"  {CYAN}cd {_REPO}/rover && \\\n"
        f"      ./target/release/coven-rover --mock "
        f"--config {config_path} --verbose{RESET}\n"
    )
    print(f"{DIM}Waiting for rover to connect…{RESET}\n")

    parser = FrameParser()

    # Step 1: send IDENTIFY_REQUEST. Assign the rover a canonical witch
    # name so the screenshots show "Hecate" or "Baba_Yaga" rather than
    # whatever happens to be in the rover's TOML. The rover's legacy
    # IDENTIFY_REQ handler picks up this name and adopts it.
    banner(f"Step 1 — Issue IDENTIFY_REQUEST (assigning '{pretty(rover_assigned_name)}')",
           BLUE_BASIC)
    dock_id = f"dock-{coven_name.lower().replace('the_', '')}"
    req = encode_identify_request(
        dock_id=dock_id,
        coven_name=coven_name,
        assigned_name=rover_assigned_name,
    )
    os.write(master_fd, req)
    log_send("IDENTIFY_REQUEST", 0x01, req)

    rover_module_id: str = rover_assigned_name

    # Drain incoming bytes and react to whatever the rover sends.
    # The Python deadline is intentionally generous; run_demo.sh kills this
    # script externally when the user-specified runtime expires.
    deadline = time.time() + 600.0
    state = "WAITING_FOR_IDENTIFY_REPLY"
    sent_verify_req = False
    sent_verify_ok = False
    sent_task_req = False
    heartbeat_count = 0

    # In-field watch state. Per the data-mule protocol, a rover in FIELD_OPS
    # is physically undocked and silent: no comms reach the dock until it
    # returns. The dock's job in this window is to *track expected silence*
    # and decide when the rover is presumed lost. We compute these bounds
    # from the round-trip waypoint distance and a conservative speed
    # estimate, then watch the wall clock.
    field_ops_start: Optional[float] = None
    expected_duration: float = 0.0
    lost_deadline: float = 0.0
    anomalous_heartbeats: int = 0
    anomalous_data_frames: int = 0
    last_watching_print: float = 0.0
    overdue_announced: bool = False
    lost_announced: bool = False
    lost_rover_name: Optional[str] = None  # set when we retire the witch name

    while time.time() < deadline:
        r, _, _ = select.select([master_fd], [], [], 0.5)
        if not r:
            continue
        data = os.read(master_fd, 4096)
        if not data:
            continue

        for msg_type, payload in parser.feed(data):
            if msg_type == 0x02:  # IDENTIFY_REPLY
                decoded = decode_identify_reply(payload) or {}
                claimed_id = decoded.get("module_id", "<unknown>")
                battery = decoded.get('battery_level', 0)
                battery_color = warn(f"battery={battery}%") if battery < 30 \
                    else ok(f"battery={battery}%")

                # Reappearance after LOST: this is a *new* rover by the
                # protocol's reckoning, even if it claims an old name. The
                # old witch name has been retired; assign a fresh one from
                # the canonical pool.
                if state == "LOST":
                    new_name = get_witch_name()
                    print()
                    print(ok(
                        f"[REAPPEAR] A rover has docked after the watch was "
                        f"closed on {pretty(lost_rover_name)}. "
                        f"Assigning fresh namespace: {witch(new_name)}."
                    ))
                    print(
                        f"           {DIM}This is treated as a new rover; "
                        f"the previous identity is not reused.{RESET}"
                    )
                    rover_module_id = new_name
                    rover_assigned_name = new_name
                    # Reset watch state for the next cycle.
                    field_ops_start = None
                    sent_verify_req = False
                    sent_task_req = False
                    overdue_announced = False
                    lost_announced = False
                    anomalous_heartbeats = 0
                    anomalous_data_frames = 0
                    state = "WAITING_FOR_IDENTIFY_REPLY"
                else:
                    rover_module_id = claimed_id

                log_recv(
                    f"IDENTIFY_REPLY from {witch(rover_module_id)} "
                    f"(caps=0x{decoded.get('capabilities', 0):02x}, {battery_color})",
                    msg_type, payload,
                )
                if state == "WAITING_FOR_IDENTIFY_REPLY":
                    banner("Step 2 — Confirm with IDENTIFY_ACK", BLUE_BASIC)
                    ack = encode_identify_ack(
                        dock_id=dock_id,
                        assigned_name=rover_module_id,
                        message=f"Welcome to {coven_name}",
                    )
                    os.write(master_fd, ack)
                    log_send("IDENTIFY_ACK", 0x05, ack)
                    state = "WAITING_FOR_VERIFY_REP"

            elif msg_type == 0x20:  # MODULE_HEARTBEAT
                decoded = decode_heartbeat(payload) or {}
                heartbeat_count += 1

                # During pre-mission phases the rover is physically docked
                # and heartbeats are exactly what we expect. Display them.
                # During IN_FIELD / OVERDUE / LOST the rover is supposed
                # to be physically undocked and silent — any heartbeat we
                # do receive is anomalous (e.g. a mock that doesn't physically
                # disconnect, or wire-level noise). Count, don't display.
                if state in ("IN_FIELD", "OVERDUE", "LOST"):
                    anomalous_heartbeats += 1
                else:
                    battery_pct = decoded.get('battery_pct', 0)
                    status_str = decoded.get('mission_status', '?')
                    if battery_pct < 10:
                        battery_str = bad(f"battery={battery_pct:.0f}%")
                    elif battery_pct < 30:
                        battery_str = warn(f"battery={battery_pct:.0f}%")
                    else:
                        battery_str = ok(f"battery={battery_pct:.0f}%")
                    print(
                        f"{MAGENTA}[ROVER→DOCK]{RESET} "
                        f"{ok(f'HEARTBEAT #{heartbeat_count}')} "
                        f"{DIM}from{RESET} {witch(rover_module_id)} "
                        f"{DIM}status={status_str}, {RESET}{battery_str}{DIM}, "
                        f"pose=({decoded.get('x', 0):.2f}, "
                        f"{decoded.get('y', 0):.2f}, "
                        f"{decoded.get('theta', 0):.2f}rad){RESET}"
                    )

                # On the third heartbeat, send the verify-OK and then a task.
                if state == "WAITING_FOR_VERIFY_REP" and heartbeat_count >= 1 and not sent_verify_req:
                    banner("Step 3 — Health check passes, send VERIFY_OK", BLUE_BASIC)
                    verify_frame = encode_verify_ok(dock_id=dock_id, module_id=rover_module_id)
                    os.write(master_fd, verify_frame)
                    log_send("VERIFY_OK", 0x03, verify_frame)
                    sent_verify_req = True
                    state = "WAITING_FOR_NORMAL"
                if state == "WAITING_FOR_NORMAL" and heartbeat_count >= 3 and not sent_task_req:
                    banner(f"Step 4 — {pretty(coven_name)} dispatches TASK_REQ to "
                           f"{pretty(rover_module_id)}", BLUE_BASIC)
                    target = {"x": 1.0, "y": 0.0}
                    task = {
                        "dock_id": dock_id,
                        "module_id": rover_module_id,
                        "task_id": f"demo-explore-{rover_module_id.lower()}-001",
                        "task": "explore",
                        "waypoints": [
                            {"x": target["x"], "y": target["y"],
                             "yaw": 0.0, "tolerance": 0.3}
                        ],
                        "dock_x": 0.0,
                        "dock_y": 0.0,
                        "coverage_threshold": 0.8,
                        "timeout": 60.0,
                    }
                    req_bytes = encode_task_request(task)
                    os.write(master_fd, req_bytes)
                    log_send(
                        f"TASK_REQ (explore → {target['x']:.1f}, "
                        f"{target['y']:.1f})", 0x10, req_bytes,
                    )
                    sent_task_req = True

                    # Compute the in-field watch bounds. Per the data-mule
                    # protocol, the rover physically undocks for the round
                    # trip and is unreachable until it returns. We track
                    # expected silence vs. presumed-lost.
                    round_trip_dist = 2.0 * (
                        target["x"] ** 2 + target["y"] ** 2
                    ) ** 0.5
                    speed_est = 0.10  # m/s — conservative mock-rover estimate
                    expected_duration = round_trip_dist / speed_est
                    lost_deadline = expected_duration * 2.5  # 250% margin

                    # `COVEN_DEMO_FORCE_LOST=1` compresses the timeline so
                    # the LOST path triggers before the mock rover returns.
                    # Useful for screenshotting the alternate (failure) flow
                    # in a single short demo run.
                    if os.environ.get("COVEN_DEMO_FORCE_LOST") == "1":
                        expected_duration = 5.0
                        lost_deadline = 12.0
                    field_ops_start = time.time()
                    last_watching_print = field_ops_start
                    state = "IN_FIELD"
                    print()
                    print(
                        f"{warn(f'[STATE] {pretty(rover_module_id)} now FIELD_OPS — physically undocked, silence expected.')}"
                    )
                    print(
                        f"        {DIM}Round trip: {round_trip_dist:.1f} m at "
                        f"{speed_est:.2f} m/s "
                        f"→ expected return in {expected_duration:.0f} s; "
                        f"presumed lost at +{lost_deadline:.0f} s.{RESET}"
                    )
                    print(
                        f"        {DIM}Heartbeats during this window are anomalous "
                        f"(no comms while undocked).{RESET}"
                    )
                    print()

            elif msg_type == 0x10:  # DATA_FRAME
                decoded = decode_data_frame(payload) or {"type": "DATA_FRAME"}
                kind = decoded.get("type", "DATA_FRAME")

                # SCAN_DATA / ODOM_DATA streams during IN_FIELD are
                # anomalous (rover should be silent). Suppress display
                # but keep a counter for the watching summary.
                if state in ("IN_FIELD", "OVERDUE", "LOST") and kind in (
                    "SCAN_DATA", "ODOM_DATA",
                ):
                    anomalous_data_frames += 1
                    continue

                log_recv(kind, msg_type, payload)
                if kind == "VERIFY_REP":
                    success = decoded.get('success')
                    failed = decoded.get('failed_checks', [])
                    success_str = ok("success=True") if success else bad("success=False")
                    failed_str = (warn(f"failed={failed}") if failed
                                  else f"{DIM}failed=[]{RESET}")
                    print(
                        f"  rover self-checks: {success_str}, {failed_str}, "
                        f"{DIM}note={decoded.get('note', '')!r}{RESET}"
                    )
                elif kind == "TASK_ACK":
                    accepted = decoded.get('success', True)
                    task_id = decoded.get('task_id')
                    if accepted:
                        print(f"  {ok(f'rover accepted task {task_id}')}")
                    else:
                        print(f"  {bad(f'rover rejected task {task_id}')}")
                elif kind == "TASK_START":
                    task_id = decoded.get('task_id')
                    print(f"  {ok(f'rover started task {task_id}')}")
                elif kind == "TASK_COMPLETE":
                    success = decoded.get('success')
                    task_id = decoded.get('task_id')
                    duration = decoded.get('duration', 0)
                    color_fn = ok if success else bad

                    if state == "LOST":
                        # The dock had already given up on this rover. A late
                        # TASK_COMPLETE under that retired identity is a
                        # protocol-level anomaly; surface it but don't let it
                        # un-retire the witch name.
                        print(warn(
                            f"  [ANOMALY] late TASK_COMPLETE for retired "
                            f"identity {pretty(rover_module_id)} "
                            f"(arrived {duration:.0f}s after dispatch, "
                            f"watch was closed at +{lost_deadline:.0f}s). "
                            f"Ignoring — name stays retired."
                        ))
                    else:
                        print()
                        print(ok(
                            f"[RETURNED] {pretty(rover_module_id)} re-docked "
                            f"and reported {color_fn('TASK_COMPLETE')} after "
                            f"{duration:.1f} s "
                            f"(expected {expected_duration:.0f} s, "
                            f"presumed-lost deadline was {lost_deadline:.0f} s)."
                        ))
                        print(
                            f"  {color_fn(f'task {task_id}')} "
                            f"{DIM}success={success}, "
                            f"coverage={decoded.get('coverage', 0):.2f}, "
                            f"duration={duration:.1f}s{RESET}"
                        )
                        if anomalous_heartbeats or anomalous_data_frames:
                            print(
                                f"  {DIM}Suppressed during in-field window: "
                                f"{anomalous_heartbeats} heartbeats, "
                                f"{anomalous_data_frames} stray data frames "
                                f"(would not exist with a real rover that "
                                f"physically undocks).{RESET}"
                            )
                        # Exit the watch — rover is back at the dock.
                        state = "NORMAL"
                        field_ops_start = None
                        sent_task_req = False
                        overdue_announced = False
                        anomalous_heartbeats = 0
                        anomalous_data_frames = 0
                        banner("Demo complete — Ctrl+C to exit", PASS)

            else:
                log_recv(f"type 0x{msg_type:02x}", msg_type, payload)

        # ----------------------------------------------------------------
        # Periodic in-field watch tick. Runs every ~5s while the rover is
        # supposed to be off in the field. Reports elapsed/expected/
        # lost-deadline; transitions IN_FIELD → OVERDUE → LOST.
        # ----------------------------------------------------------------
        now = time.time()
        if state in ("IN_FIELD", "OVERDUE") and field_ops_start is not None \
                and now - last_watching_print >= 5.0:
            elapsed = now - field_ops_start
            remaining = lost_deadline - elapsed
            anomaly_note = ""
            if anomalous_heartbeats or anomalous_data_frames:
                anomaly_note = (
                    f" {DIM}({anomalous_heartbeats} anomalous heartbeats, "
                    f"{anomalous_data_frames} stray data frames suppressed){RESET}"
                )

            if elapsed >= lost_deadline:
                if not lost_announced:
                    print()
                    print(bad(
                        f"[LOST] {pretty(rover_module_id)} presumed lost — "
                        f"{elapsed:.0f} s without re-dock (250% of expected "
                        f"{expected_duration:.0f} s)."
                    ))
                    print(
                        f"       {DIM}Witch name '{pretty(rover_module_id)}' "
                        f"retired and released back to the canonical pool. "
                        f"Heartbeat watch closed.{RESET}"
                    )
                    print(
                        f"       {DIM}If a rover re-docks now, it receives a "
                        f"fresh witch name (new namespace).{RESET}"
                    )
                    lost_rover_name = rover_module_id
                    release_witch_name(rover_module_id)
                    lost_announced = True
                    state = "LOST"
            elif elapsed > expected_duration:
                if not overdue_announced:
                    print()
                    print(warn(
                        f"[OVERDUE] {pretty(rover_module_id)} past expected "
                        f"return ({elapsed:.0f} s > {expected_duration:.0f} s). "
                        f"Lost in {remaining:.0f} s."
                    ) + anomaly_note)
                    overdue_announced = True
                    state = "OVERDUE"
                else:
                    print(warn(
                        f"[OVERDUE] {pretty(rover_module_id)} elapsed "
                        f"{elapsed:.0f}/{expected_duration:.0f} s. Lost in "
                        f"{remaining:.0f} s."
                    ) + anomaly_note)
            else:
                print(warn(
                    f"[WATCHING] {pretty(rover_module_id)} in field. "
                    f"elapsed {elapsed:.0f}/{expected_duration:.0f} s. "
                    f"Lost in {remaining:.0f} s."
                ) + anomaly_note)
            last_watching_print = now

    print(f"{YELLOW}Demo deadline reached. Exiting.{RESET}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Interrupted.{RESET}")
        sys.exit(0)
