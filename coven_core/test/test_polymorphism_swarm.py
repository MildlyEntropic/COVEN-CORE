#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
test_polymorphism_swarm.py — Heterogeneous-swarm dispatch and data-flow tests.

These tests exercise the central polymorphism claim of the COVEN protocol on
an extended class hierarchy: a swarm of 8 mock rovers with three distinct
payload types (LiDAR, spectrometer, barometer) coordinates through a single
dock running unmodified protocol, dispatch, and transport code.

The swarm composition pins the heterogeneity:

    4 × MappingRover     payload=LIDAR        caps=0x03
    2 × SpectralRover    payload=SPECTROMETER caps=0x13
    2 × BarometerRover   payload=BAROMETER    caps=0x41

The barometer payload is a deliberate extension added after the original
four-class hierarchy was specified. Adding it required only matrix-and-enum
extensions in task_auctioneer.py (PayloadType.BAROMETER, TaskType.BAROMETRIC,
new row/column in PAYLOAD_TASK_COMPATIBILITY) — the dispatch algorithm in
RoverInfo.calculate_bid was not modified. This is the open/closed principle
at the matrix layer: extend the data, leave the algorithm alone.

Two test classes:

  TestSwarmDispatch — runs randomized auctions over the 8-rover swarm and
    asserts every dispatch is capability-appropriate. The dock never inspects
    rover concrete identity; it dispatches based on declared capabilities and
    the payload-task compatibility matrix.

  TestSensorAgnosticTransport — each rover type produces its native sensor
    data as opaque bytes ("LIDAR_RANGES_MOCK", "SPECTROMETER_MOCK_DATA",
    "BAROMETRIC: 1013.25 hPa"), runs it through the production frame builder
    and chunk assembler, and asserts the bytes survive round-trip unmodified.
    The transport layer carries arbitrary sensor payloads without interpreting
    them — sensor_type tags the format, but the bytes themselves are opaque.

Author: Alexander Shultis
Date: April 2026
"""

import random
import struct
import sys
import unittest
from collections import Counter
from pathlib import Path

# Make the dock-side package importable when running this file directly.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coven_core.frame_codec import (  # noqa: E402
    BatchChunkAssembler,
    CHUNK_TYPE_DATA,
    CHUNK_TYPE_HEADER,
    SENSOR_TYPE_BAROMETER,
    SENSOR_TYPE_LIDAR,
    SENSOR_TYPE_SPECTROMETER,
)
from coven_core.task_auctioneer import (  # noqa: E402
    Mission,
    PayloadType,
    RoverInfo,
    RoverStatus,
    TaskType,
)


# Capability bit layout (per IDENTIFY_REPLY documentation in dock_uart.rs)
CAP_ENCODERS = 0x01
CAP_LIDAR = 0x02
CAP_SPECTROMETER = 0x10
CAP_DRILL = 0x20
CAP_BAROMETER = 0x40  # Reserved bit, allocated for the barometer extension


DOCK = (0.0, 0.0)


def make_rover(
    module_id: str,
    payload: PayloadType,
    capabilities: int,
    battery_pct: float = 85.0,
) -> RoverInfo:
    """Construct an idle rover with the given payload, capabilities, and battery."""
    return RoverInfo(
        module_id=module_id,
        status=RoverStatus.IDLE,
        payload=payload,
        capabilities=capabilities,
        battery_pct=battery_pct,
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


# ---------------------------------------------------------------------------
# Swarm composition: 8 rovers, 3 payload types, 5 distinct module_ids per type.
# ---------------------------------------------------------------------------

SWARM_SPEC = (
    # 4 MappingRovers (LiDAR + encoders)
    [(f"map_{i}", PayloadType.LIDAR, CAP_ENCODERS | CAP_LIDAR) for i in range(4)]
    # 2 SpectralRovers (spectrometer + LiDAR + encoders)
    + [
        (f"spec_{i}", PayloadType.SPECTROMETER, CAP_ENCODERS | CAP_LIDAR | CAP_SPECTROMETER)
        for i in range(2)
    ]
    # 2 BarometerRovers (barometer + encoders, no LiDAR)
    + [
        (f"baro_{i}", PayloadType.BAROMETER, CAP_ENCODERS | CAP_BAROMETER)
        for i in range(2)
    ]
)


def build_swarm():
    """Construct the heterogeneous 8-rover swarm."""
    return [make_rover(name, payload, caps) for (name, payload, caps) in SWARM_SPEC]


# ===========================================================================
# Test class 1 — randomized swarm dispatch
# ===========================================================================


class TestSwarmDispatch(unittest.TestCase):
    """Heterogeneous swarm dispatch under randomized task allocation."""

    def test_swarm_composition_is_heterogeneous(self):
        """Swarm contains rovers from 3 distinct payload types."""
        swarm = build_swarm()
        payloads = {r.payload for r in swarm}
        self.assertEqual(
            payloads,
            {PayloadType.LIDAR, PayloadType.SPECTROMETER, PayloadType.BAROMETER},
            "swarm should span 3 payload types",
        )
        self.assertEqual(len(swarm), 8, "swarm should have 8 rovers")

    def test_capability_bitmasks_match_class_intent(self):
        """Each rover's capability bitmask matches its class spec."""
        swarm = build_swarm()
        for rover in swarm:
            if rover.module_id.startswith("map_"):
                self.assertEqual(rover.capabilities, 0x03, f"{rover.module_id} caps")
                self.assertTrue(rover.capabilities & CAP_LIDAR)
            elif rover.module_id.startswith("spec_"):
                self.assertEqual(rover.capabilities, 0x13, f"{rover.module_id} caps")
                self.assertTrue(rover.capabilities & CAP_SPECTROMETER)
                self.assertTrue(rover.capabilities & CAP_LIDAR)
            elif rover.module_id.startswith("baro_"):
                self.assertEqual(rover.capabilities, 0x41, f"{rover.module_id} caps")
                self.assertTrue(rover.capabilities & CAP_BAROMETER)
                self.assertFalse(
                    rover.capabilities & CAP_LIDAR,
                    "BarometerRover deliberately has no LiDAR bit",
                )

    def test_randomized_dispatch_always_capability_appropriate(self):
        """100 randomized auctions across 3 task types — every winner is capable."""
        rng = random.Random(42)  # reproducible
        swarm = build_swarm()
        task_types = [TaskType.EXPLORE, TaskType.SPECTRAL, TaskType.BAROMETRIC]

        winners_by_task = Counter()
        for trial in range(100):
            task = rng.choice(task_types)
            mission = make_mission(task)

            # Run the auction: every rover bids; lowest finite bid wins.
            bids = [(r, r.calculate_bid(mission, DOCK)) for r in swarm]
            capable = [(r, b) for r, b in bids if b < 999999]

            self.assertGreater(
                len(capable),
                0,
                f"trial {trial} ({task.value}): no rover capable; "
                f"all bids: {[(r.module_id, b) for r, b in bids]}",
            )

            winner_rover, winner_bid = min(capable, key=lambda x: x[1])
            winners_by_task[(task, winner_rover.module_id.split('_')[0])] += 1

            # Capability appropriateness: the winning rover's payload-task
            # entry must be a finite (non-exclusion) value.
            from coven_core.task_auctioneer import PAYLOAD_TASK_COMPATIBILITY
            matrix_value = PAYLOAD_TASK_COMPATIBILITY[winner_rover.payload][task]
            self.assertLess(
                matrix_value,
                999999,
                f"trial {trial}: winner {winner_rover.module_id} "
                f"({winner_rover.payload.value}) has matrix exclusion for {task.value}",
            )

            # Specific exclusions: SPECTRAL must go to a SPECTROMETER payload;
            # BAROMETRIC must go to a BAROMETER payload. EXPLORE can go to
            # any LiDAR-capable rover (LiDAR or SPECTROMETER payloads).
            if task == TaskType.SPECTRAL:
                self.assertEqual(
                    winner_rover.payload,
                    PayloadType.SPECTROMETER,
                    f"trial {trial}: SPECTRAL routed to non-spectrometer "
                    f"{winner_rover.module_id} ({winner_rover.payload.value})",
                )
            elif task == TaskType.BAROMETRIC:
                self.assertEqual(
                    winner_rover.payload,
                    PayloadType.BAROMETER,
                    f"trial {trial}: BAROMETRIC routed to non-barometer "
                    f"{winner_rover.module_id} ({winner_rover.payload.value})",
                )
            elif task == TaskType.EXPLORE:
                # LiDAR-capable rovers are preferred. With the swarm above,
                # MappingRovers (LiDAR) have base bid -25, SpectralRovers
                # (SPECTROMETER) have +10, BarometerRovers (BAROMETER) have
                # +30. So MappingRovers should win EXPLORE.
                self.assertTrue(
                    winner_rover.module_id.startswith("map_"),
                    f"trial {trial}: EXPLORE should prefer MappingRover, "
                    f"got {winner_rover.module_id} ({winner_rover.payload.value})",
                )

        # Distribution sanity: every task type was sampled.
        sampled_tasks = {task for task, _ in winners_by_task.keys()}
        self.assertEqual(sampled_tasks, set(task_types), "all task types should appear")

    def test_no_dock_modification_required_for_barometer(self):
        """Adding the barometer class required no calculate_bid modification.

        This test asserts the structural property: BarometerRover dispatches
        through the same calculate_bid code path as every other rover. The
        bid is computed from PAYLOAD_TASK_COMPATIBILITY (a table) and the
        capability bitmask (a byte), without any concrete-type branch in
        the algorithm. The auctioneer never inspects whether a rover is a
        barometer; it reads the rover's declared payload and capabilities.
        """
        baro = make_rover("baro_test", PayloadType.BAROMETER, 0x41)
        # Native task: bid must be finite (capability-compatible).
        bid_native = baro.calculate_bid(make_mission(TaskType.BAROMETRIC), DOCK)
        self.assertLess(bid_native, 999999)
        # Cross-class task: bid must be 999999 (capability-incompatible).
        bid_cross = baro.calculate_bid(make_mission(TaskType.SPECTRAL), DOCK)
        self.assertEqual(bid_cross, 999999)
        # The same calculate_bid that handles MappingRover handles BarometerRover.
        # No new dispatch function exists.
        self.assertEqual(
            baro.calculate_bid.__qualname__,
            "RoverInfo.calculate_bid",
            "BarometerRover must use the same dispatch method as every other rover",
        )

    def test_open_closed_at_matrix_layer(self):
        """The dispatch algorithm did not change when BAROMETER was added.

        Verify by inspecting the source: calculate_bid reads
        PAYLOAD_TASK_COMPATIBILITY by indexing rover.payload and
        mission.task_type. It does not contain a branch on PayloadType.BAROMETER
        or any other concrete type — the algorithm consults the table.
        """
        import inspect
        from coven_core.task_auctioneer import RoverInfo as _RoverInfo
        source = inspect.getsource(_RoverInfo.calculate_bid)
        # The algorithm must not branch on the new payload type.
        self.assertNotIn(
            "BAROMETER",
            source,
            "calculate_bid must not contain a BAROMETER-specific branch; "
            "polymorphism requires the algorithm to read from the matrix",
        )
        # Sanity: it does consult the matrix.
        self.assertIn(
            "PAYLOAD_TASK_COMPATIBILITY",
            source,
            "calculate_bid should read from the payload-task matrix",
        )


# ===========================================================================
# Test class 2 — sensor-agnostic transport
# ===========================================================================


def build_header_chunk(
    batch_id: int,
    total_samples: int,
    total_chunks: int,
    module_id: str,
    mission_id: str,
    sensor_type: int,
) -> bytes:
    """Construct a CHUNK_TYPE_HEADER chunk payload for the given rover/sensor.

    Mirrors the wire format the Rust firmware emits in encode_data_batch_frames.
    """
    data = bytearray()
    data.extend(struct.pack('<I', batch_id))
    data.extend(struct.pack('<I', total_samples))   # u32
    data.extend(struct.pack('<H', total_chunks))    # u16
    mid = module_id.encode('utf-8')
    data.append(len(mid))
    data.extend(mid)
    msn = mission_id.encode('utf-8')
    data.append(len(msn))
    data.extend(msn)
    # batch metadata: mission_start(8) + wheel_radius_mm(2) + wheel_base_mm(2) + ticks_per_rev(2)
    data.extend(struct.pack('<d', 0.0))  # mission_start
    data.extend(struct.pack('<H', 80))   # wheel_radius_mm
    data.extend(struct.pack('<H', 298))  # wheel_base_mm
    data.extend(struct.pack('<H', 816))  # ticks_per_rev
    # sensor: type(1) + config_len(2) + config
    data.append(sensor_type)
    data.extend(struct.pack('<H', 0))  # no sensor config in mock
    return bytes([CHUNK_TYPE_HEADER]) + bytes(data)


def build_data_chunk(
    batch_id: int,
    chunk_seq: int,
    samples: list,
) -> bytes:
    """Construct a CHUNK_TYPE_DATA chunk with the given samples."""
    data = bytearray()
    data.extend(struct.pack('<I', batch_id))
    data.extend(struct.pack('<H', chunk_seq))
    data.extend(struct.pack('<H', len(samples)))
    for s in samples:
        data.extend(struct.pack('<d', s["timestamp"]))
        data.extend(struct.pack('<i', s["left_ticks"]))
        data.extend(struct.pack('<i', s["right_ticks"]))
        sd = s["sensor_data"]
        data.extend(struct.pack('<H', len(sd)))
        data.extend(sd)
    return bytes([CHUNK_TYPE_DATA]) + bytes(data)


class TestSensorAgnosticTransport(unittest.TestCase):
    """The transport carries arbitrary sensor payloads without interpretation."""

    def _round_trip(self, sensor_type: int, sensor_data: bytes, module_id: str):
        """Build header + data chunks, feed through assembler, return assembled batch."""
        assembler = BatchChunkAssembler()
        batch_id = 0xDEADBEEF & 0xFFFFFFFF
        sample = {
            "timestamp": 1.0,
            "left_ticks": 100,
            "right_ticks": 102,
            "sensor_data": sensor_data,
        }
        header = build_header_chunk(
            batch_id=batch_id,
            total_samples=1,
            total_chunks=1,
            module_id=module_id,
            mission_id="m-poly-001",
            sensor_type=sensor_type,
        )
        # Header alone returns None.
        self.assertIsNone(assembler.feed_chunk(header))
        data = build_data_chunk(batch_id=batch_id, chunk_seq=0, samples=[sample])
        result = assembler.feed_chunk(data)
        self.assertIsNotNone(result, "assembler should complete after final chunk")
        return result

    def test_lidar_rover_payload_round_trips(self):
        """A MappingRover's LiDAR mock data round-trips opaquely through the transport."""
        mock_data = b"LIDAR_RANGES_MOCK"
        result = self._round_trip(SENSOR_TYPE_LIDAR, mock_data, "map_0")
        self.assertEqual(result["sensor_type"], SENSOR_TYPE_LIDAR)
        self.assertEqual(result["module_id"], "map_0")
        self.assertEqual(len(result["batch"]["samples"]), 1)
        self.assertEqual(result["batch"]["samples"][0]["sensor_data"], mock_data)

    def test_spectrometer_rover_payload_round_trips(self):
        """A SpectralRover's spectrometer mock data round-trips opaquely."""
        mock_data = b"SPECTROMETER_MOCK_DATA: peaks at 656.3nm 486.1nm"
        result = self._round_trip(SENSOR_TYPE_SPECTROMETER, mock_data, "spec_0")
        self.assertEqual(result["sensor_type"], SENSOR_TYPE_SPECTROMETER)
        self.assertEqual(result["batch"]["samples"][0]["sensor_data"], mock_data)

    def test_barometer_rover_payload_round_trips(self):
        """A BarometerRover's pressure mock data round-trips opaquely.

        This is the load-bearing test for sensor-agnostic transport: a sensor
        type added after the original protocol was specified, with payload
        bytes the protocol has never seen, must still survive the wire layer
        unmodified. The transport doesn't know what a barometer is. It carries
        bytes.
        """
        mock_data = b"BAROMETRIC: 1013.25 hPa @ 295.15K"
        result = self._round_trip(SENSOR_TYPE_BAROMETER, mock_data, "baro_0")
        self.assertEqual(result["sensor_type"], SENSOR_TYPE_BAROMETER)
        self.assertEqual(result["batch"]["samples"][0]["sensor_data"], mock_data)

    def test_arbitrary_bytes_round_trip(self):
        """Even adversarial payloads (zeros, all-FF, mixed) round-trip."""
        for label, payload in [
            ("all_zeros", b"\x00" * 16),
            ("all_ff", b"\xff" * 16),
            ("alternating", bytes(range(256))),
            ("text_with_zero", b"BAROMETRIC\x00READING"),
        ]:
            with self.subTest(payload=label):
                # Use barometer sensor type for this — payload bytes are opaque.
                result = self._round_trip(SENSOR_TYPE_BAROMETER, payload, "baro_x")
                got = result["batch"]["samples"][0]["sensor_data"]
                self.assertEqual(got, payload, f"{label}: payload corrupted in transport")

    def test_dock_distinguishes_per_rover_namespace(self):
        """Three rovers from three classes upload concurrently; per-rover state is isolated."""
        # Each rover gets its own assembler — namespace isolation in production
        # is per-rover-bridge, not a shared queue. Verify each rover's data
        # ends up tagged with its own module_id and sensor_type.
        cases = [
            ("map_0", SENSOR_TYPE_LIDAR, b"LIDAR_RANGES_MOCK"),
            ("spec_0", SENSOR_TYPE_SPECTROMETER, b"SPECTROMETER_MOCK_DATA"),
            ("baro_0", SENSOR_TYPE_BAROMETER, b"BAROMETRIC: 1013.25 hPa"),
        ]
        results = [self._round_trip(st, sd, mid) for (mid, st, sd) in cases]
        # Each result's module_id and sensor_type tag the source correctly.
        for (expected_mid, expected_st, expected_sd), result in zip(cases, results):
            self.assertEqual(result["module_id"], expected_mid)
            self.assertEqual(result["sensor_type"], expected_st)
            self.assertEqual(result["batch"]["samples"][0]["sensor_data"], expected_sd)


if __name__ == "__main__":
    unittest.main(verbosity=2)
