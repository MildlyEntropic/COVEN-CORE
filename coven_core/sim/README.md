# COVEN Simulation Bridge

This directory contains the Gazebo simulation assets recovered from the
December 2025 pre-Rust era, plus the bridge that adapts them to the
post-pivot architecture.

## Architecture

```
   Gazebo Harmonic                     ROS2 Jazzy                   COVEN Protocol
+----------------------+        +-----------------------+      +-----------------------+
|  coven_world         |        |                       |      |                       |
|  └─ coven_rover      |        |  sim_rover_proxy.py   |      |  rover_bridge.py      |
|     ├ /scan          | ─────> |  (one per rover)      |      |  (unmodified prod     |
|     ├ /odometry      |        |                       |      |   dock-side code)     |
|     └ /cmd_vel       | <────  |  ┌─ subscribers ──┐   |      |                       |
+----------------------+        |  └─ encoders ─────┴─> |  PTY |   ─── opens slave ───>|
                                |  ┌─ FrameParser ──┐   | pair |                       |
                                |  └─ FSM driver ───┴── |  ─── |                       |
                                +-----------------------+      +-----------------------+
                                                                          │
                                                                  Same auctioneer,
                                                                  bridge_protocol,
                                                                  SLAM Toolbox, etc.
                                                                  as runs against
                                                                  real Pi Zero rovers.
```

The `sim_rover_proxy` node is a one-rover-equivalent process that:

1. **Owns a PTY pair** (`os.openpty()`) and exposes the slave path (or a
   stable symlink at `/tmp/coven_sim_uart`) for the dock to connect to.
2. **Subscribes** to the rover's Gazebo-bridged sensor topics
   (`/<rover>/scan` and `/<rover>/odometry`).
3. **Walks the COVEN FSM** (BOOT → IDENTIFY → WAIT_VERIFY → NORMAL →
   FIELD_OPS) using the same wire format the Rust firmware emits, via
   `coven_core/rover_codec.py` (verified byte-for-byte by 15 unit tests
   in `test/test_rover_codec.py`).
4. **Streams real-time sensor data** to the dock as `DATA_FRAME` subtype
   `0x20` JSON envelopes (`ScanData`, `OdomData`) at 5 Hz / 10 Hz.
5. **Heartbeats** at 1 Hz with current pose and battery.
6. **On `TASK_REQ`** drives the simulated rover toward the waypoint by
   publishing `/<rover>/cmd_vel` until the goal is reached, then sends
   `TaskComplete`.

The dock cannot tell a sim-proxied rover from a Rust-firmware Pi Zero
rover at the protocol layer — by design.

## What's in this directory

```
sim/
├── README.md                  ← this file
├── worlds/
│   ├── coven_test.sdf         ← single-rover sandbox (recovered Dec 2025)
│   └── coven_4rover.sdf       ← four-rover world (recovered Dec 2025)
├── models/
│   ├── coven_rover/model.sdf  ← diff-drive rover w/ gpu_lidar + IMU
│   ├── dock.sdf               ← dock model
│   └── module.sdf             ← payload module
├── coven_core/
│   └── world_generator.py     ← randomized obstacle/wall generator
└── launch/
    ├── coven_1rover_sim.launch.py            ← original Dec 2025 single-rover
    ├── coven_2rover_sim.launch.py            ← original Dec 2025 dual-rover
    ├── coven_4rover_sim.launch.py            ← original Dec 2025 four-rover
    ├── coven_data_mule_sim.launch.py         ← data-mule-architecture sim
    ├── coven_dock_centric_sim.launch.py      ← dock-centric variant
    ├── coven_dock_centric_2rover_sim.launch.py
    ├── coven_navigation.launch.py            ← Nav2 stack per rover
    ├── slam_toolbox_namespaced.launch.py     ← per-rover SLAM
    └── wait_for_topic.py                     ← topic-readiness barrier
```

The Dec 2025 launch files are preserved here verbatim for reference —
they were ROS2-native (rovers were ROS2 nodes pre-Rust pivot) and run
against the *old* coven_core architecture. The post-pivot single-rover
sim launch lives at `coven_core/launch/coven_sim_proxy_1rover.launch.py`
(one level up from this directory).

## How to run (in the sim Docker container)

```bash
# 1. Build the sim Docker image (Dockerfile now installs Gazebo Harmonic,
#    ros_gz bridges, Nav2, SLAM Toolbox, and socat).
cd Code/COVEN-CORE/coven_core
docker build -t coven-sim -f docker/Dockerfile .

# 2. Run the container with X forwarding for Gazebo GUI.
docker run --rm -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/..:/ros2_ws/src/COVEN-CORE \
    --network host \
    coven-sim

# 3. Inside the container, build the workspace and source it.
cd /ros2_ws
colcon build --packages-select coven_core
source install/setup.bash

# 4. Launch the single-rover sim.
ros2 launch coven_core coven_sim_proxy_1rover.launch.py

# Or headless:
ros2 launch coven_core coven_sim_proxy_1rover.launch.py headless:=true
```

What you should see:
- Gazebo opens with a single rover at the origin in `coven_test.sdf`.
- Logs from `sim_rover_proxy` showing it received `IDENTIFY_REQUEST`,
  replied with `IDENTIFY_REPLY` + capabilities `0x03`, transitioned
  through `WAIT_VERIFY` to `NORMAL`, and is heartbeating at 1 Hz.
- Logs from `rover_bridge` showing the rover registered, ID confirmed,
  health verified, ready for tasks.
- `ros2 topic echo /witch_morgan/scan` should show LaserScan messages
  flowing from Gazebo.
- The dock can issue a `TASK_REQ` (e.g., via `frontier_dispatcher` or a
  manual ROS2 service call) and the simulated rover will physically
  drive in Gazebo toward the waypoint.

## Verifying the bridge speaks correctly without ROS2

Before booting the full sim stack, you can verify the wire-format codec
locally with `unittest`:

```bash
cd Code/COVEN-CORE/coven_core
python3 -m unittest test.test_rover_codec
# Expected: Ran 15 tests in 0.004s — OK
```

This tests every rover→dock message type round-trips through the dock's
production decoders — meaning the proxy speaks bytes the dock will
accept. No ROS2, no Gazebo, no Docker required.

## Limitations of the sim bridge

What the sim bridge **does** validate:
- Wire format end-to-end (proxy encodes, dock decodes, both round-trip).
- COVEN FSM behavior under realistic message ordering.
- Capability-based dispatch (the dock dispatches the proxy's declared
  capabilities exactly as it would for a real rover).
- Multi-rover concurrent coordination (run N proxies, point N PTYs at
  the dock — the dock handles them as N concurrent rovers).
- Sensor-data flow through the data-mule pipeline.
- Live `cmd_vel` round-trip (dock → proxy → Gazebo → robot moves).

What the sim bridge **does not** validate:
- Real Pi Zero 2W performance characteristics.
- Real RPLIDAR packet parsing, real motor PWM, real encoder interrupts.
- Real-world physical effects (LiDAR glints, motor stalls, battery sag).
- Real EMI on USB cables; real serial timing under thermal load.

See thesis §5.5.1 ("Physical Hardware Validation Deferred") for the
honest framing of which claims this campaign supports and which still
require physical hardware deployment.
