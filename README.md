# COVEN

**Composable Operations for Versatile Exploration Networks**

A finite state machine protocol for coordinating autonomous exploration modules with docking infrastructure. Designed for multi-robot planetary exploration missions.

---

## What It Does

- **Dock-Module Architecture** - Central dock coordinates multiple exploration modules
- **Automatic Discovery** - Modules self-register via identification handshake
- **Health Verification** - Sensor checks before mission assignment (LiDAR, odometry)
- **Competitive Task Bidding** - Modules bid on missions based on capability/proximity
- **Fault Tolerance** - Graceful degradation when modules fail
- **Frontier Exploration** - Autonomous SLAM-based mapping with Nav2

Built for lunar exploration research, warehouse automation, and multi-robot coordination studies.

---

## Quick Start

### Using the `coven` CLI (Recommended)

```bash
cd ~/ros2_ws

# Run tests
./coven test

# Launch full simulation (Gazebo + Nav2 + COVEN)
./coven sim          # Single robot
./coven sim 3        # Three robots

# Manual node control
./coven dock         # Start dock node
./coven module       # Start module node

# See all commands
./coven help
```

### Manual Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select coven_core --symlink-install
source install/setup.bash
```

---

## Simulation

Launch a complete multi-robot simulation with Gazebo, SLAM, Nav2, and COVEN:

```bash
# Single robot (default)
ros2 launch coven_core coven_multi_sim.launch.py

# Multiple robots
ros2 launch coven_core coven_multi_sim.launch.py num_robots:=3

# Different world
ros2 launch coven_core coven_multi_sim.launch.py num_robots:=2 world:=depot
```

This starts:
- Gazebo simulation (warehouse environment)
- One COVEN dock node
- N TurtleBot4 robots with SLAM + Nav2
- N COVEN module nodes (one per robot)

Each robot operates in its own namespace (`/robot_1/`, `/robot_2/`, etc.) while COVEN protocol topics remain global (`/coven/*`).

---

## Architecture

```
                    ┌─────────────────┐
                    │   COVEN DOCK    │
                    │  (Coordinator)  │
                    └────────┬────────┘
                             │
              Global /coven/* topics
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
    ┌────▼─────┐       ┌────▼─────┐       ┌────▼─────┐
    │ Module 1 │       │ Module 2 │       │ Module N │
    │ robot_1  │       │ robot_2  │       │ robot_N  │
    └────┬─────┘       └────┬─────┘       └────┬─────┘
         │                   │                   │
    /robot_1/*          /robot_2/*          /robot_N/*
    (sensors, nav)      (sensors, nav)      (sensors, nav)
```

### Protocol State Machine

**Module States:**
```
BOOT → IDENTIFY → WAIT_VERIFY → NORMAL ⟷ FIELD_OPS
                       ↓
                   REJECTED (on health check failure)
```

**Message Flow:**
```
Dock                          Module
  │                              │
  │──── IDENTIFY_REQ (broadcast) │
  │                              │
  │◄─── IDENTIFY_REP ────────────│  "I'm Hermione_Granger, ReconRover"
  │                              │
  │──── VERIFY_REQ ──────────────│
  │                              │
  │◄─── VERIFY_REP ──────────────│  "LiDAR: OK, Odom: OK"
  │                              │
  │──── ENABLE_12V ──────────────│
  │                              │
  │◄─── HEARTBEAT (1.25 Hz) ─────│
  │                              │
  │──── BID_NOTICE ──────────────│  "Explore sector A"
  │                              │
  │◄─── BID_PROPOSAL ────────────│  "I can reach it in 45s"
  │                              │
  │──── TASK_REQ ────────────────│  "You win, go explore"
  │                              │
  │◄─── TASK_ACK ────────────────│
  │◄─── TASK_START ──────────────│
  │         ...exploration...    │
  │◄─── TASK_COMPLETE ───────────│  "Done, 73% coverage"
```

### Topics

**Discovery & Verification:**
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/coven/identify_req` | Dock → All | Request module identification |
| `/coven/identify_rep` | Module → Dock | Report ID, type, capabilities |
| `/coven/verify_req` | Dock → Module | Request health check |
| `/coven/verify_rep` | Module → Dock | Report sensor status |
| `/coven/enable_12v` | Dock → Module | Enable power (after verification) |

**Operations:**
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/coven/heartbeat` | Module → Dock | Periodic status (0.8s interval) |
| `/coven/bid_notice` | Dock → All | Announce available task |
| `/coven/bid_proposal` | Module → Dock | Submit bid for task |
| `/coven/task_req` | Dock → Module | Assign task to winner |
| `/coven/task_ack` | Module → Dock | Acknowledge assignment |
| `/coven/task_start` | Module → Dock | Report task started |
| `/coven/task_complete` | Module → Dock | Report task finished |

All messages are JSON-encoded `std_msgs/String`.

---

## Module Naming

Modules are automatically assigned names from curated lists:

**Modules (Witches):** Hermione_Granger, Elphaba, Circe, Baba_Yaga, Wanda_Maximoff, Kiki, Yubaba, Mother_Talzin, etc. (30 names)

**Docks (Covens):** The_Sanderson_Sisters, The_Weird_Sisters, The_Bene_Gesserit, The_Hex_Girls, etc. (10 names)

Names are randomly assigned without duplicates until the pool is exhausted.

---

## Testing

### Quick Tests
```bash
./coven test-quick     # Unit tests only (~3s)
./coven test           # Full rigorous suite
./coven test-all       # Everything including stress tests
```

### Test Coverage

| Suite | Tests | Coverage |
|-------|-------|----------|
| `test_common.py` | 23 | Message encoding, naming system |
| `test_fsm_transitions.py` | 17 | State machine logic |
| `test_protocol_validation.py` | 10 | Full protocol flows |
| `test_rigorous_integration.py` | 9 | Performance benchmarks |

**59 tests total**, all passing.

---

## Dependencies

### Core (Required)
```bash
sudo apt install ros-humble-desktop python3-pytest
```

### Simulation (For `coven sim`)
```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot4-simulator
```

---

## Project Structure

```
coven_core/
├── coven_core/
│   ├── dock_node.py       # Dock FSM, module coordination, task bidding
│   ├── module_node.py     # Module FSM, health checks, task execution
│   ├── exploration.py     # Frontier-based autonomous navigation
│   └── common.py          # Protocol messages, naming system
├── launch/
│   └── coven_multi_sim.launch.py  # Multi-robot Gazebo simulation
├── test/
│   ├── test_common.py
│   ├── test_fsm_transitions.py
│   ├── test_protocol_validation.py
│   └── test_rigorous_integration.py
└── README.md
```

---

## Configuration

### Module Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_namespace` | `""` | Robot namespace for sensors (e.g., `robot_1`) |
| `skip_health_check` | `false` | Skip LiDAR/odom verification |
| `use_sim_time` | `false` | Use simulation clock |

### Timing Constants

```python
HB_PERIOD = 0.8           # Heartbeat interval (seconds)
HB_MISS_THRESHOLD = 3     # Missed heartbeats before timeout
IDENTIFY_INTERVAL = 5.0   # Discovery broadcast interval
EXPLORATION_TIMEOUT = 300 # Max exploration time (seconds)
COVERAGE_THRESHOLD = 0.8  # Target map coverage (80%)
```

---

## Status

### Complete
- Multi-module coordination (tested with 5+ concurrent modules)
- Discovery and registration protocol
- Health verification (LiDAR, odometry)
- Competitive task bidding
- Heartbeat monitoring with timeout detection
- State machine transitions
- Frontier-based exploration with Nav2
- Multi-robot TF namespace isolation
- Graceful degradation on module failure

### In Development
- Hardware fabrication (CubeRover-scale modules)
- Physical docking mechanism
- Map merging from multiple explorers

---

## Research Context

COVEN addresses the "coordination gap" in planetary robotics - the absence of a standardized protocol enabling heterogeneous robots to work together without mission-specific pre-configuration.

**Key References:**
- HOTDOCK (DFKI, 2018) - Electromechanical docking
- CADRE (NASA JPL, 2024) - Swarm coordination
- RIMRES (Cordes et al., 2010) - Reconfigurable multi-robot systems
- Nav2 (Macenski et al., 2020) - Autonomous navigation
- Frontier Exploration (Yamauchi, 1997) - Autonomous mapping

---

## License

MIT License - See LICENSE file

---

## Author

Alexander Shultis
University of Hawaiʻi at Mānoa
Department of Astronomy
December 2025

Faculty Advisor: Dr. Miguel Nunes (HSFL)
Thesis Advisor: Dr. Jiaoyang Zhu (Colorado School of Mines)

---

**ROS2 Humble | Python 3.10+ | Gazebo Ignition**
