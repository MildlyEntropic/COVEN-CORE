#!/bin/bash
# SPDX-License-Identifier: MIT
#
# run_overnight_demo.sh — wrapper for overnight_demo.py.
#
# Usage:
#   coven-overnight-demo                  # run until Ctrl+C
#   coven-overnight-demo --hours 8        # 8 hours then summarize
#   coven-overnight-demo --interval 30    # mission every ~30s
#   coven-overnight-demo --hours 8 --seed 42   # reproducible 8h run
#
# Author: Alexander Shultis
# Date: April 2026

set -u
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO"
exec python3 -u scripts/overnight_demo.py "$@"
