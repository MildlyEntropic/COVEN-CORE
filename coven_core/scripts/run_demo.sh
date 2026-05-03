#!/bin/bash
# SPDX-License-Identifier: MIT
#
# run_demo.sh — One-shot end-to-end COVEN demo for screenshots.
#
# Spawns the demo dock (Python) and the rover firmware (Rust) in mock mode,
# wires them together via a virtual UART, and lets them complete a full
# COVEN handshake. Both sides log to stdout (color-coded).
#
# Requirements (none of which need ROS2/Gazebo/Docker):
#   - python3 with the coven_core repo on the path (this script handles it)
#   - cargo on PATH (Rust toolchain) so the firmware can build if needed
#   - socat installed (for stable PTY symlinking on systems where bare
#     PTY paths don't work with tokio-serial)
#
# Usage:
#   coven-demo                  # default: 45s, happy path (rover returns)
#   coven-demo 60               # 60s runtime, happy path
#   coven-demo lose             # ~30s, force LOST timeline (rover presumed lost)
#   coven-demo lose 25          # 25s, LOST flow
#
# Author: Alexander Shultis
# Date: April 2026

set -u

# Locate the repo root regardless of where this script is invoked from.
# Resolve symlinks so a `~/.local/bin/coven-demo` shortcut still finds the
# real repo path instead of resolving relative to the symlink's directory.
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
ROVER_BIN="$REPO/rover/target/release/coven-rover"

# Parse mode + runtime. First arg may be the literal word "lose" to force
# the LOST timeline; remaining numeric arg sets runtime.
DEMO_MODE="happy"
if [ "${1:-}" = "lose" ] || [ "${1:-}" = "lost" ]; then
    DEMO_MODE="lose"
    shift
fi

if [ "$DEMO_MODE" = "lose" ]; then
    RUNTIME="${1:-30}"
    export COVEN_DEMO_FORCE_LOST=1
else
    RUNTIME="${1:-45}"
fi

if [ ! -x "$ROVER_BIN" ]; then
    echo "ERROR: rover binary not found at $ROVER_BIN"
    echo "Build it first:"
    echo "    cd $REPO/rover && cargo build --release"
    exit 1
fi

# Add cargo to PATH if it's installed via puccinialin (common on Fedora).
if [ -d "$HOME/.cache/puccinialin/cargo/bin" ]; then
    export PATH="$HOME/.cache/puccinialin/cargo/bin:$PATH"
fi

# Cleanup state from any previous run.
pgrep -f "demo_dock_for_screenshots\|coven-rover --mock" 2>/dev/null \
    | xargs -r kill -9 2>/dev/null
rm -f /tmp/coven_demo_rover.toml /tmp/coven_demo_uart /tmp/demo_dock.log

DOCK_LOG=/tmp/coven_demo_dock.log
ROVER_LOG=/tmp/coven_demo_rover.log

echo "=================================================================="
echo "COVEN End-to-End Demo"
echo "=================================================================="
echo "Repo:    $REPO"
echo "Rover:   $ROVER_BIN"
echo "Runtime: ${RUNTIME}s"
echo "Mode:    $DEMO_MODE  $([ "$DEMO_MODE" = "lose" ] && echo "(timeline compressed: expected=5s, lost=12s)")"
echo ""

# --- Stage 1: start the demo dock ---
echo ">>> Starting demo dock (Python)…"
# python3 -u disables stdout buffering so the dock log streams in real time.
( cd "$REPO" && python3 -u scripts/demo_dock_for_screenshots.py ) \
    > "$DOCK_LOG" 2>&1 &
DOCK_PID=$!

# Wait for the dock to create the symlink (max 5s).
for _ in $(seq 1 50); do
    if [ -L /tmp/coven_demo_uart ] && [ -f /tmp/coven_demo_rover.toml ]; then
        break
    fi
    sleep 0.1
done

if [ ! -L /tmp/coven_demo_uart ]; then
    echo "ERROR: demo dock did not produce /tmp/coven_demo_uart in 5s."
    echo "Dock log:"
    cat "$DOCK_LOG"
    kill -9 $DOCK_PID 2>/dev/null
    exit 1
fi

# Resolve the PTY slave path (some serial libraries don't follow symlinks).
PTY_PATH="$(readlink /tmp/coven_demo_uart)"
echo "    PTY slave: $PTY_PATH"
echo "    Symlink:   /tmp/coven_demo_uart"
echo "    Config:    /tmp/coven_demo_rover.toml"
echo ""

# Patch the config to use the real PTY path (works around tokio-serial's
# refusal to open via symlink in some versions).
sed -i "s|port = \"/tmp/coven_demo_uart\"|port = \"$PTY_PATH\"|" \
    /tmp/coven_demo_rover.toml

# --- Stage 2: start the rover, run for $RUNTIME seconds ---
echo ">>> Starting rover firmware (mock mode, verbose)…"
( "$ROVER_BIN" --mock --verbose --config /tmp/coven_demo_rover.toml ) \
    > "$ROVER_LOG" 2>&1 &
ROVER_PID=$!

echo "    Rover PID: $ROVER_PID  (logs: $ROVER_LOG)"
echo "    Dock  PID: $DOCK_PID   (logs: $DOCK_LOG)"
echo ""
echo ">>> Demo running for ${RUNTIME}s. Both sides log to stdout below."
echo "=================================================================="

# Wait for the runtime, then tear down.
sleep "$RUNTIME"

# Stop both. Rover gets SIGTERM first so the FSM can shut down cleanly.
kill -TERM $ROVER_PID 2>/dev/null
kill -TERM $DOCK_PID 2>/dev/null
sleep 1
kill -KILL $ROVER_PID 2>/dev/null
kill -KILL $DOCK_PID 2>/dev/null

echo ""
echo "=================================================================="
echo "DOCK SIDE OUTPUT (Python demo dock)"
echo "=================================================================="
cat "$DOCK_LOG"

echo ""
echo "=================================================================="
echo "ROVER SIDE OUTPUT (Rust firmware in mock mode)"
echo "=================================================================="
cat "$ROVER_LOG"

echo ""
echo "=================================================================="
echo "Demo complete."
echo "  Dock log:  $DOCK_LOG"
echo "  Rover log: $ROVER_LOG"
echo "=================================================================="
