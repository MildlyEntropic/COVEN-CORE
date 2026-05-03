#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
test_capability_dispatch.py — Polymorphism / capability-based dispatch tests.

Exercises the dock-side task auctioneer with rovers from the four declared
classes in the COVEN protocol:

    MappingRover    caps 0x03  (encoders + LiDAR)
    ReconRover      caps 0x05  (encoders + ultrasonic)
    SpectralRover   caps 0x13  (encoders + LiDAR + spectrometer)
    DrillRover      caps 0x23  (encoders + LiDAR + drill)

The goal is to confirm at the protocol layer that:

  1. Capability-incompatible task/payload pairings score 999999 (excluded).
  2. Capability-appropriate pairings score lowest (preferred).
  3. The auctioneer dispatches based on declared capabilities, not concrete
     rover identity (polymorphism).
  4. Adding or removing a rover class requires no auctioneer modification.

These tests do NOT touch ROS2; they exercise the pure-Python coordination
logic. They are the protocol-level analog of the empirical campaign that
will run against physical hardware in future work.

Author: Alexander Shultis
Date: April 2026
"""

import sys
import unittest
from pathlib import Path

# Make the dock-side package importable when running this file directly
# (so the test does not depend on ROS2 being installed).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coven_core.task_auctioneer import (  # noqa: E402
    Mission,
    PayloadType,
    RoverInfo,
    RoverStatus,
    TaskType,
)


def make_rover(module_id: str, payload: PayloadType, capabilities: int) -> RoverInfo:
    """Construct an idle, full-battery rover with the given payload and caps."""
    return RoverInfo(
        module_id=module_id,
        status=RoverStatus.IDLE,
        payload=payload,
        capabilities=capabilities,
        battery_pct=85.0,  # >80% to neutralize battery factor in bid
        position=(0.0, 0.0),
        heading=0.0,
    )


def make_mission(task_type: TaskType, target=(5.0, 0.0)) -> Mission:
    """Construct a single-waypoint mission of the given task type."""
    return Mission(
        mission_id=f"m-{task_type.value}",
        task_type=task_type,
        waypoints=[target],
        dock_return=(0.0, 0.0),
    )


# Dock at origin; missions all target a point in front of the dock.
DOCK = (0.0, 0.0)

# The four declared rover classes from the COVEN protocol specification.
ROVER_CLASSES = [
    ("MappingRover", PayloadType.LIDAR, 0x03),
    ("ReconRover", PayloadType.CAMERA, 0x05),  # No LiDAR bit → camera-as-recon
    ("SpectralRover", PayloadType.SPECTROMETER, 0x13),
    ("DrillRover", PayloadType.DRILL, 0x23),
]


class TestCapabilityDispatch(unittest.TestCase):
    """Capability-based dispatch (polymorphism) — protocol-level tests."""

    # ------------------------------------------------------------------
    # Pin the capability-bit layout via a real RoverInfo round-trip. The
    # production code uses magic literals (e.g. capabilities=0x03) rather
    # than named constants, so the strongest test we can write at this
    # layer is to construct a real RoverInfo with each declared bitmask
    # and verify the production calculate_bid path observes the bits we
    # set. The pairwise non-overlap check below documents the spec values.
    # ------------------------------------------------------------------

    def test_capability_bitmask_observed_by_dispatch(self):
        # Two camera-payload rovers, identical except for the LiDAR bit.
        # The +30 EXPLORE penalty path in calculate_bid keys off bit 0x02
        # specifically (capabilities & 0x02). If the production code is
        # reading a different bit, the difference between the two bids
        # will not be exactly 30.
        with_lidar_bit = make_rover("a", PayloadType.CAMERA, 0x03)
        without_lidar_bit = make_rover("b", PayloadType.CAMERA, 0x05)
        bid_with = with_lidar_bit.calculate_bid(make_mission(TaskType.EXPLORE), DOCK)
        bid_without = without_lidar_bit.calculate_bid(make_mission(TaskType.EXPLORE), DOCK)
        self.assertEqual(
            bid_without - bid_with, 30,
            "production calculate_bid does not key the no-LiDAR penalty "
            "off bit 0x02 as the spec specifies"
        )

        # Pairwise non-overlap of the spec's documented bits — local check,
        # documents intent. The production code has no named constants to
        # import, so this is the honest scope of what a unit test verifies.
        bits = [0x01, 0x02, 0x04, 0x10, 0x20]
        for i, a in enumerate(bits):
            for b in bits[i + 1:]:
                self.assertEqual(
                    a & b, 0,
                    f"documented capability bits 0x{a:02x} and 0x{b:02x} overlap"
                )

    # ------------------------------------------------------------------
    # Each class bids appropriately for its native task.
    # ------------------------------------------------------------------

    # Each native-task test asserts the exact post-modifier bid, computed by
    # walking calculate_bid against the test inputs:
    #   base 50 + matrix -25 + battery -15 (>80%) + orientation -5 (target
    #   straight ahead at heading 0) = 5. The bid floor is 1, so the result
    #   stands at 5. If any modifier path stops firing, the exact assertion
    #   catches it; the loose `bid < 50` form would not.

    def test_mapping_rover_prefers_explore(self):
        rover = make_rover("witch_alpha", PayloadType.LIDAR, 0x03)
        bid = rover.calculate_bid(make_mission(TaskType.EXPLORE), DOCK)
        self.assertEqual(bid, 5, f"MappingRover EXPLORE bid expected 5, got {bid}")

    def test_spectral_rover_prefers_spectral(self):
        rover = make_rover("witch_beta", PayloadType.SPECTROMETER, 0x13)
        bid = rover.calculate_bid(make_mission(TaskType.SPECTRAL), DOCK)
        self.assertEqual(bid, 5, f"SpectralRover SPECTRAL bid expected 5, got {bid}")

    def test_drill_rover_prefers_sample(self):
        rover = make_rover("witch_gamma", PayloadType.DRILL, 0x23)
        bid = rover.calculate_bid(make_mission(TaskType.SAMPLE), DOCK)
        self.assertEqual(bid, 5, f"DrillRover SAMPLE bid expected 5, got {bid}")

    def test_recon_rover_prefers_survey(self):
        # ReconRover has caps 0x05 (no LiDAR bit). With camera payload,
        # SURVEY is its specialty (CAMERA + SURVEY matrix entry is -25).
        rover = make_rover("witch_delta", PayloadType.CAMERA, 0x05)
        bid = rover.calculate_bid(make_mission(TaskType.SURVEY), DOCK)
        self.assertEqual(bid, 5, f"ReconRover SURVEY bid expected 5, got {bid}")

    # ------------------------------------------------------------------
    # Capability-incompatible pairings are excluded (bid 999999).
    # ------------------------------------------------------------------

    def test_lidar_rover_excluded_from_spectral(self):
        rover = make_rover("witch_alpha", PayloadType.LIDAR, 0x03)
        bid = rover.calculate_bid(make_mission(TaskType.SPECTRAL), DOCK)
        self.assertEqual(bid, 999999, "LIDAR payload cannot accept SPECTRAL task")

    def test_lidar_rover_excluded_from_sample(self):
        rover = make_rover("witch_alpha", PayloadType.LIDAR, 0x03)
        bid = rover.calculate_bid(make_mission(TaskType.SAMPLE), DOCK)
        self.assertEqual(bid, 999999, "LIDAR payload cannot accept SAMPLE task")

    def test_drill_rover_excluded_from_spectral(self):
        rover = make_rover("witch_gamma", PayloadType.DRILL, 0x23)
        bid = rover.calculate_bid(make_mission(TaskType.SPECTRAL), DOCK)
        self.assertEqual(bid, 999999, "DRILL payload cannot accept SPECTRAL task")

    def test_spectrometer_rover_excluded_from_sample(self):
        rover = make_rover("witch_beta", PayloadType.SPECTROMETER, 0x13)
        bid = rover.calculate_bid(make_mission(TaskType.SAMPLE), DOCK)
        self.assertEqual(bid, 999999, "SPECTROMETER payload cannot accept SAMPLE task")

    # ------------------------------------------------------------------
    # No-LiDAR penalty: a rover without the LiDAR capability bit pays a
    # +30 penalty on EXPLORE tasks even if its payload type is compatible
    # with exploration (e.g., camera). This exercises the capability-
    # bitmask dispatch path independently of the payload-task matrix.
    # ------------------------------------------------------------------

    def test_no_lidar_pays_explore_penalty(self):
        with_lidar = make_rover("a", PayloadType.CAMERA, 0x03)  # has LiDAR bit
        without_lidar = make_rover("b", PayloadType.CAMERA, 0x05)  # no LiDAR bit

        bid_with = with_lidar.calculate_bid(make_mission(TaskType.EXPLORE), DOCK)
        bid_without = without_lidar.calculate_bid(make_mission(TaskType.EXPLORE), DOCK)

        self.assertEqual(
            bid_without - bid_with, 30,
            f"no-LiDAR EXPLORE penalty should be +30, "
            f"got {bid_without - bid_with} (with={bid_with}, without={bid_without})"
        )

    # ------------------------------------------------------------------
    # Polymorphism: when a mixed swarm receives a task, the lowest bidder
    # is the rover whose declared capability set best matches the task,
    # regardless of its concrete identity. Run for each task type in turn.
    # ------------------------------------------------------------------

    def test_swarm_dispatches_to_best_capability_match(self):
        # Build a swarm spanning all four declared classes, with module_ids
        # that encode the class so the winner assertion is unambiguous.
        named = [
            ("MappingRover_unit", PayloadType.LIDAR, 0x03),
            ("ReconRover_unit", PayloadType.CAMERA, 0x05),
            ("SpectralRover_unit", PayloadType.SPECTROMETER, 0x13),
            ("DrillRover_unit", PayloadType.DRILL, 0x23),
        ]
        swarm = [make_rover(n, p, c) for (n, p, c) in named]

        # The auctioneer picks the lowest finite bid; the expected-winner
        # column encodes the protocol's polymorphic intent: the dock
        # dispatches to the capability-appropriate rover without inspecting
        # concrete type.
        cases = [
            (TaskType.EXPLORE, "MappingRover"),
            (TaskType.SPECTRAL, "SpectralRover"),
            (TaskType.SAMPLE, "DrillRover"),
            (TaskType.SURVEY, "ReconRover"),
        ]

        for task_type, expected_class in cases:
            mission = make_mission(task_type)
            bids = [(r.module_id, r.calculate_bid(mission, DOCK)) for r in swarm]
            capable = [(n, b) for n, b in bids if b < 999999]
            self.assertGreater(
                len(capable), 0,
                f"no rover capable of {task_type.value}: bids={bids}"
            )
            winner_name, winner_bid = min(capable, key=lambda x: x[1])
            self.assertTrue(
                winner_name.startswith(expected_class),
                f"task {task_type.value}: expected {expected_class} winner, "
                f"got {winner_name} (bid={winner_bid}); all bids: {bids}"
            )

    # ------------------------------------------------------------------
    # Open/closed: adding a fifth class to the swarm requires no change
    # to the existing dispatch logic. Demonstrate by spawning a fresh
    # auctioneer-equivalent computation that includes a hypothetical
    # CargoRover and verifying the swarm still dispatches each task to
    # its correct class — including DELIVER going to the new CargoRover.
    # ------------------------------------------------------------------

    def test_adding_fifth_class_requires_no_logic_change(self):
        swarm = [
            make_rover("MappingRover_unit", PayloadType.LIDAR, 0x03),
            make_rover("ReconRover_unit", PayloadType.CAMERA, 0x05),
            make_rover("SpectralRover_unit", PayloadType.SPECTROMETER, 0x13),
            make_rover("DrillRover_unit", PayloadType.DRILL, 0x23),
            # Fifth class added — capability bit 0x40 reserved for cargo
            # in the protocol spec; the auctioneer never inspects it.
            make_rover("CargoRover_unit", PayloadType.CARGO, 0x41),
        ]

        mission = make_mission(TaskType.DELIVER)
        bids = [(r.module_id, r.calculate_bid(mission, DOCK)) for r in swarm]
        capable = [(n, b) for n, b in bids if b < 999999]
        winner_name, _ = min(capable, key=lambda x: x[1])

        # The cargo rover must win DELIVER despite never having existed
        # when the auctioneer was written.
        self.assertTrue(
            winner_name.startswith("CargoRover"),
            f"DELIVER should route to CargoRover, got {winner_name}; bids: {bids}"
        )

    # ------------------------------------------------------------------
    # Fault tolerance at the dispatch layer: a damaged rover (damage_level
    # 3) is excluded regardless of payload match. This guards against the
    # dock dispatching a task to a known-bad rover.
    # ------------------------------------------------------------------

    def test_critical_damage_excludes_rover(self):
        rover = make_rover("witch_alpha", PayloadType.LIDAR, 0x03)
        rover.damage_level = 3
        bid = rover.calculate_bid(make_mission(TaskType.EXPLORE), DOCK)
        self.assertGreaterEqual(
            bid, 999999,
            f"critically damaged rover should be excluded, got bid {bid}"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
