#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
dispatch_task_cli.py — `coven-dispatch-task`: manually issue a mission
to the dock auctioneer from the command line.

Use cases:
  * Demonstrate the polymorphism claim live during thesis defense.
  * Trigger a task during a sim run when frontier_dispatcher isn't running.
  * Smoke-test a freshly-deployed dock by sending it a known waypoint.

The dock's rover_bridge subscribes to `/coven/mission_request` and feeds
incoming missions into the TaskAuctioneer (which dispatches by capability
bitmask, not concrete rover identity). This CLI publishes one well-formed
mission to that topic and exits.

Usage:
    coven-dispatch-task --task-type explore --waypoint 5,0 --waypoint 5,5
    coven-dispatch-task --task-type spectral --waypoint 3,2
    coven-dispatch-task --task-type barometric --waypoint 0,8 --priority 5

Author: Alexander Shultis
Date: April 2026
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:  # pragma: no cover
    HAS_ROS2 = False
    rclpy = None  # type: ignore
    Node = object  # type: ignore


VALID_TASK_TYPES = {
    "explore",
    "spectral",
    "sample",
    "deliver",
    "survey",
    "barometric",
    "excavate",
    "haul",
    "aerial_survey",
}


def _parse_waypoint(s: str) -> tuple:
    parts = s.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            f"waypoint must be 'x,y' (got {s!r})"
        )
    try:
        return (float(parts[0]), float(parts[1]))
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"invalid waypoint {s!r}: {e}")


def _build_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="coven-dispatch-task",
        description=(
            "Publish a single mission to /coven/mission_request. The dock "
            "auctioneer dispatches it to a capability-appropriate rover."
        ),
    )
    parser.add_argument(
        "--task-type",
        required=True,
        choices=sorted(VALID_TASK_TYPES),
        help="Mission task type. The auctioneer uses this against the "
             "PAYLOAD_TASK_COMPATIBILITY matrix to score bids.",
    )
    parser.add_argument(
        "--waypoint",
        type=_parse_waypoint,
        action="append",
        required=True,
        help="Waypoint as 'x,y' (meters from dock origin). Pass repeatedly "
             "for multi-waypoint missions.",
    )
    parser.add_argument(
        "--priority",
        type=int,
        default=0,
        help="Mission priority. Higher = more urgent. Default 0.",
    )
    parser.add_argument(
        "--mission-id",
        default=None,
        help="Override the auto-generated mission_id. Default: "
             "<task-type>_<uuid8>.",
    )
    parser.add_argument(
        "--topic",
        default="/coven/mission_request",
        help="Override the publish topic. Default: /coven/mission_request",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the JSON that would be published without contacting ROS2.",
    )
    return parser


def main(argv=None) -> int:
    parser = _build_args()
    args = parser.parse_args(argv)

    mission_id = (
        args.mission_id
        or f"{args.task_type}_{uuid.uuid4().hex[:8]}"
    )
    payload = {
        "mission_id": mission_id,
        "task_type": args.task_type,
        "waypoints": [{"x": x, "y": y} for (x, y) in args.waypoint],
        "priority": args.priority,
    }
    json_str = json.dumps(payload)

    if args.dry_run:
        print(json_str)
        return 0

    if not HAS_ROS2:
        print(
            "ERROR: rclpy is not installed. Run this inside the COVEN sim "
            "Docker container, or pass --dry-run to inspect the payload.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = rclpy.create_node("coven_dispatch_task_cli")
    pub = node.create_publisher(String, args.topic, 10)

    # Wait briefly for the publisher to register against any subscribers.
    # Without this, the dock's rover_bridge can miss the message.
    deadline = time.time() + 2.0
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = String()
    msg.data = json_str
    pub.publish(msg)
    node.get_logger().info(
        f"Published mission {mission_id} ({args.task_type}, "
        f"{len(args.waypoint)} waypoints) to {args.topic}"
    )

    # Allow the message to leave the queue before we tear down.
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
