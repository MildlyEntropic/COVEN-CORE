#!/bin/bash
# SPDX-License-Identifier: MIT
#
# run_swarm_demo.sh — wrapper for swarm_demo.py.
#
# Usage:
#   coven-swarm-demo             # 60s default
#   coven-swarm-demo 90          # custom runtime
#
# The swarm demo is pure Python: 8 mock rovers as threads, each owning a
# PTY pair, all speaking the COVEN wire format via coven_core.rover_codec.
# No Rust binary, no Docker, no ROS2 needed.
#
# Author: Alexander Shultis
# Date: April 2026

set -u

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNTIME="${1:-60}"

cd "$REPO"
exec python3 -u scripts/swarm_demo.py "$RUNTIME"
