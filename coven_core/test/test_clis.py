#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
test_clis.py — exercise the operator CLIs without requiring ROS2.

`coven-dispatch-task --dry-run` builds the JSON payload that would be
published to /coven/mission_request and prints it to stdout. We verify
the payload is valid JSON, contains every field the dock's
_mission_request_callback inspects, and rejects malformed waypoints.

`coven-ping`'s argparse surface is exercised similarly. The actual serial
I/O cannot run here (no PTY paired with anything), but we verify --help
exits 0 and unknown args are rejected.

Author: Alexander Shultis
Date: April 2026
"""

import io
import json
import sys
import unittest
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coven_core import dispatch_task_cli, ping_cli  # noqa: E402


def run_cli(main_fn, argv):
    """Run a CLI's main(argv) capturing stdout/stderr and exit code."""
    out = io.StringIO()
    err = io.StringIO()
    code = None
    try:
        with redirect_stdout(out), redirect_stderr(err):
            code = main_fn(argv)
    except SystemExit as e:
        code = e.code
    return code, out.getvalue(), err.getvalue()


class TestDispatchTaskCLI(unittest.TestCase):
    def test_dry_run_emits_valid_json(self):
        code, out, _ = run_cli(
            dispatch_task_cli.main,
            ["--task-type", "explore",
             "--waypoint", "5,0",
             "--waypoint", "5,5",
             "--dry-run"],
        )
        self.assertEqual(code, 0)
        payload = json.loads(out.strip())
        self.assertEqual(payload["task_type"], "explore")
        self.assertEqual(len(payload["waypoints"]), 2)
        self.assertEqual(payload["waypoints"][0], {"x": 5.0, "y": 0.0})
        self.assertEqual(payload["waypoints"][1], {"x": 5.0, "y": 5.0})
        self.assertEqual(payload["priority"], 0)
        self.assertTrue(payload["mission_id"].startswith("explore_"))

    def test_dry_run_barometric_task(self):
        # Confirms the BAROMETRIC task type added in the swarm extension
        # is accepted by the CLI surface (and downstream by the dock,
        # which now also accepts it in _mission_request_callback).
        code, out, _ = run_cli(
            dispatch_task_cli.main,
            ["--task-type", "barometric",
             "--waypoint", "0,8",
             "--priority", "5",
             "--dry-run"],
        )
        self.assertEqual(code, 0)
        payload = json.loads(out.strip())
        self.assertEqual(payload["task_type"], "barometric")
        self.assertEqual(payload["priority"], 5)

    def test_dry_run_custom_mission_id(self):
        code, out, _ = run_cli(
            dispatch_task_cli.main,
            ["--task-type", "spectral",
             "--waypoint", "1,1",
             "--mission-id", "demo-001",
             "--dry-run"],
        )
        self.assertEqual(code, 0)
        payload = json.loads(out.strip())
        self.assertEqual(payload["mission_id"], "demo-001")

    def test_invalid_waypoint_rejected(self):
        code, _, err = run_cli(
            dispatch_task_cli.main,
            ["--task-type", "explore",
             "--waypoint", "not_a_waypoint",
             "--dry-run"],
        )
        self.assertNotEqual(code, 0)
        self.assertIn("waypoint", err.lower())

    def test_invalid_task_type_rejected(self):
        code, _, err = run_cli(
            dispatch_task_cli.main,
            ["--task-type", "telekinesis",
             "--waypoint", "1,1",
             "--dry-run"],
        )
        self.assertNotEqual(code, 0)
        # argparse uses 'invalid choice' for choices=
        self.assertIn("invalid choice", err.lower())

    def test_missing_waypoint_rejected(self):
        code, _, err = run_cli(
            dispatch_task_cli.main,
            ["--task-type", "explore", "--dry-run"],
        )
        self.assertNotEqual(code, 0)
        self.assertIn("waypoint", err.lower())


class TestPingCLI(unittest.TestCase):
    def test_help_exits_zero(self):
        code, out, _ = run_cli(ping_cli.main, ["--help"])
        self.assertEqual(code, 0)
        self.assertIn("coven-ping", out)
        self.assertIn("SYSTEM_PING", out)

    def test_missing_port_rejected(self):
        code, _, err = run_cli(ping_cli.main, [])
        self.assertNotEqual(code, 0)
        # argparse error mentions the missing positional 'port'.
        self.assertIn("port", err.lower())

    def test_unknown_arg_rejected(self):
        code, _, err = run_cli(ping_cli.main, ["--no-such-flag", "/dev/null"])
        self.assertNotEqual(code, 0)
        self.assertIn("unrecognized", err.lower())


if __name__ == "__main__":
    unittest.main(verbosity=2)
