# Screenshot recipe for COVEN demos

## 4. Overnight realistic-physics swarm — the "leave it running" demo

```bash
coven-overnight-demo --hours 8 > ~/coven-overnight.log 2>&1
# go to bed
# come back
less -R ~/coven-overnight.log     # scroll the night
tail -80 ~/coven-overnight.log    # the closing summary
```

**What this swarm models:**

- **9 heterogeneous rovers**, exactly the spec'd composition: 2 LiDAR + 2 Spectrometer + 2 Barometer + 1 Excavator + 1 Hauler + 1 Optical Drone.
- **Per-rover physics**: each payload type has a distinct max speed (drones at 8 m/s, excavators at 0.20 m/s), battery capacity, drain rate, recharge rate, and on-site overhead factor. `ROVER_PHYSICS` table in `overnight_demo.py`.
- **Per-task on-site work time**: aerial flights are quick (~30 s), excavation is slow (~15 min). `TASK_WORK_TIME` table.
- **Predicted duration is physics-derived**: `(2 × distance_to_waypoint) / rover.speed + work_time × rover.overhead_factor`. So a 10-m round-trip excavation takes ~32 minutes for an excavator and ~3 seconds of flight + work for the drone.
- **Actual duration is a clustered random factor** of predicted: 95% in [0.99, 1.05], 4% moderate spread [0.90, 1.50], 1% heavy tail [1.50, 3.00]. The 1% tail crosses the dock's 250% LOST deadline — so the failure-handling path triggers organically rather than being scripted.
- **Battery management**: rovers drain in field, recharge while docked, refuse to bid below 15%. The auctioneer's existing penalty also kicks in below 30% battery (production code, unchanged).
- **Concurrent in-field rovers**: dispatches don't wait. Multiple rovers can be in field at once. A real-time watcher prints `[RETURNED]` events asynchronously between mission lines.
- **Mission deferral**: when all capable rovers are in field or below battery threshold, the mission is deferred and logged (a real auctioneer would queue it).
- **Graceful failure**: a heavy-tail mission triggers `[LOST]`, retires the witch name, releases it back to the canonical pool, spawns a replacement rover with a fresh witch name, and rejoins the swarm — all in one frame.

**The closing summary** (the headline screenshot for §5.4.2) shows:

- Mission outcomes: on-time / late / lost / deferred totals
- Field-time per mission: avg + range
- Mission mix histogram across all task types served
- Dispatched-payload histogram (which rover types actually won bids)
- Per-rover scoreboard with completed + lost counts and `[in field]` markers
- Rover turnover log (retired → replaced)
- The economic argument: per-dispatch cost calculation, Decadal Survey framing

**Default cadence is 90s** between mission generations, giving multiple rovers in the field simultaneously. Override with `--interval` for faster or slower demos.

**Reproducibility**: pass `--seed N` for reproducible runs.

---



Three independent things that produce screenshot-worthy output, in
increasing order of effort. None of #1 or #2 requires Docker, ROS2,
or Gazebo.

## 1. The full test campaign passing — the easiest screenshot

```bash
cd Code/COVEN-CORE/coven_core
python3 -m unittest \
    test.test_frame_codec \
    test.test_capability_dispatch \
    test.test_polymorphism_swarm \
    test.test_rover_codec \
    test.test_clis -v
```

What you get on screen: every test name printed, each ending in `ok`,
followed by `Ran 99 tests in 0.0XXs — OK`. Pair with the Rust side:

```bash
cd Code/COVEN-CORE/coven_core/rover
cargo test --release
```

→ `test result: ok. 31 passed; 0 failed; 0 ignored; 0 measured`. **130
tests total, all green.** This is the headline screenshot for §4.

For a single-frame summary instead of per-test verbose output, drop
the `-v` flag and capture the bottom three lines of each suite.

## 2. Live COVEN protocol exchange — the "it actually runs" screenshot

This is the demo I'd bring to defense. Real production Rust firmware
in mock mode handshaking with a fake dock over a virtual UART, all on
your laptop, no Docker.

```bash
cd Code/COVEN-CORE/coven_core
./scripts/run_demo.sh 25
```

The script prints both sides of the wire side-by-side. You'll see, in
order:

1. `IDENTIFY_REQUEST (27 bytes on wire)` ← dock → rover
2. `IDENTIFY_REPLY (caps=0x03, battery=100.0%)` ← rover → dock
3. `IDENTIFY_ACK` ← dock → rover
4. `HEARTBEAT #1 status=STARTUP, pose=(0.00, 0.00, 0.00rad)` ← rover → dock
5. `VERIFY_OK` → `VERIFY_REP (success=True, note='Mock mode - all checks pass')`
6. Heartbeats with `status=IDLE` (rover entered NORMAL state)
7. `TASK_REQ (explore → 1.0, 0.0)` → `TASK_ACK` → `TASK_START`
8. Heartbeats with `status=ACTIVE` and pose progressing toward (1.0, 0.0)
9. Streaming `ODOM_DATA` and `SCAN_DATA` frames

**Three screenshots that justify the thesis:**

- **Frame 1** — the IDENTIFY/VERIFY handshake. Crop the `Step 1` through
  `Step 3` block. Shows the rover walking BOOT → IDENTIFY → WAIT_VERIFY →
  NORMAL with real protocol bytes on the wire.

- **Frame 2** — the TASK_REQ dispatch. Crop the `Step 4` block plus the
  first 5–6 heartbeats after it. Shows the dock dispatching, the rover
  acknowledging, transitioning to FIELD_OPS, and the pose actually
  changing as the mock navigation drives it toward the goal.

- **Frame 3** — the rover-side log. Open `/tmp/coven_demo_rover.log` and
  filter:
  ```bash
  grep -E "Identify|Verify|Heartbeat|state transition|Received|FieldOps|TASK" \
      /tmp/coven_demo_rover.log
  ```
  This shows the rover's FSM transitions in production-firmware terms
  (`State -> WaitVerify`, `received TASK_REQ`, `State -> FieldOps`).

To re-run cleanly: `./scripts/run_demo.sh` (defaults to 25s) — old logs
are wiped each run.

## 3. The Gazebo sim with the proxy bridge — the optional flashy screenshot

This needs the sim Docker container, which is the only path that requires
Gazebo + ROS2:

```bash
cd Code/COVEN-CORE/coven_core
docker build -t coven-sim -f docker/Dockerfile .
./docker/run.sh sim
```

You'll get the Gazebo GUI showing one diff-drive rover at the origin in
`coven_test.sdf`, plus terminal output from the proxy showing the same
COVEN handshake as #2 — but this time the heartbeat pose is real Gazebo
physics, the LaserScan is real (simulated) RPLIDAR data, and a TASK_REQ
will physically drive the simulated robot in the Gazebo world.

If Gazebo doesn't start (X forwarding, etc.), use:
```bash
./docker/run.sh sim-headless
```

You won't get the GUI but you'll still see the proxy logs proving the
dock dispatched a barometric task to a barometer rover or whatever
scenario you set up.

## Quick screenshots without running anything

The thesis already has empirical results in §4. If you just need a
screenshot of the test campaign output for the slide deck, re-run #1
into a file and use that:

```bash
cd Code/COVEN-CORE/coven_core
{ echo "===PYTHON==="
  python3 -m unittest \
      test.test_frame_codec test.test_capability_dispatch \
      test.test_polymorphism_swarm test.test_rover_codec test.test_clis 2>&1
  echo
  echo "===RUST==="
  (cd rover && cargo test --release 2>&1 | tail -5)
} | tee /tmp/coven_test_summary.txt
```

`/tmp/coven_test_summary.txt` is now a single file ready to screenshot or
embed into a slide.
