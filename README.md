# COVEN

**Collaborative Observation Vehicles for Exploration Networks**

A ROS2 multi-robot exploration system using the **data mule** pattern for efficient SLAM in resource-constrained environments.

---

## What It Does

COVEN deploys multiple lightweight rovers ("witches") from a central dock ("coven") to explore unknown environments. Unlike traditional multi-robot SLAM where each robot builds its own map, COVEN rovers are simple data collectors—they record sensor data during exploration and return it to the dock for centralized processing.

```
                    ┌─────────────────┐
                    │     DOCK        │
                    │  (The Coven)    │
                    │                 │
                    │  ┌───────────┐  │
                    │  │  Offline  │  │
                    │  │   SLAM    │  │
                    │  └───────────┘  │
                    │  ┌───────────┐  │
                    │  │ Frontier  │  │
                    │  │ Dispatch  │  │
                    │  └───────────┘  │
                    └────────┬────────┘
                             │
            ┌────────────────┼────────────────┐
            │                │                │
            ▼                ▼                ▼
       ┌─────────┐      ┌─────────┐      ┌─────────┐
       │ Witch 1 │      │ Witch 2 │      │ Witch N │
       │ (Rover) │      │ (Rover) │      │ (Rover) │
       └─────────┘      └─────────┘      └─────────┘
```

Built for lunar exploration research, warehouse automation, and multi-robot coordination studies.

---

## Why Data Mules?

Traditional multi-robot SLAM requires:
- Powerful onboard computers for each robot
- Complex inter-robot communication for map merging
- Significant bandwidth for sharing map data

The data mule approach instead:
- Uses simple, cheap rovers with minimal compute
- Records raw sensor data during exploration
- Returns to dock for centralized processing
- Enables offline SLAM with full computational resources

This makes COVEN ideal for:
- Swarm robotics with many low-cost units
- Communication-denied environments (caves, lunar pits)
- Scenarios where map quality matters more than real-time updates

---

## Quick Start

### Simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select coven_core
source install/setup.bash

# Launch with 2 rovers (default)
ros2 launch coven_core coven_data_mule_sim.launch.py

# Launch with 4 rovers
ros2 launch coven_core coven_data_mule_sim.launch.py witch_count:=4
```

This opens separate terminal windows for:
- **Gazebo**: Simulation environment
- **Witch terminals**: One per rover showing navigation status
- **Coven terminal**: Dispatcher and SLAM processor status

---

## Architecture

### Core Nodes

| Node | Location | Purpose |
|------|----------|---------|
| `data_mule_module` | Rover | Navigate, record sensors, return data |
| `frontier_dispatcher` | Dock | Analyze map, assign exploration targets |
| `offline_slam_processor` | Dock | Replay recorded data through SLAM |

### Rover State Machine

```
IDLE → RECORDING → RETURNING → TRANSFERRING → COMPLETE → IDLE
  │                                                        │
  └────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Dispatch**: Dock sends waypoint to idle rover
2. **Explore**: Rover navigates toward target, recording LiDAR + odometry at 10Hz
3. **Return**: Rover heads back to dock when target reached
4. **Transfer**: Rover saves JSON data file, notifies dock
5. **Process**: Offline SLAM replays data at 10x speed, updates map
6. **Repeat**: Frontier analysis finds new targets, cycle continues

### Topics

**Published by Rovers:**
| Topic | Purpose |
|-------|---------|
| `/coven/module_status` | Rover heartbeats and state |
| `/coven/mule_data` | Data transfer notifications |

**Published by Dock:**
| Topic | Purpose |
|-------|---------|
| `/coven/missions` | Mission assignments |
| `/coven/dispatcher_status` | Exploration progress |
| `/coven/slam_processor_status` | SLAM processing state |

---

## Data Output

Session data is saved to:
```
~/Desktop/COVEN/Data/YYYYMMDD.HHMM.SS/
└── Coven_Name/
    ├── Witch_Name/
    │   └── Scan[SS:SS].json    # Recorded sensor frames
    └── SLAM/
        ├── map_mission_001.pgm  # Incremental maps
        ├── map_mission_001.yaml
        └── map.pgm              # Final map on shutdown
```

Each `Scan[SS:SS].json` contains:
- Mission metadata (ID, module, timestamps)
- Initial position (x, y, theta) for world-frame alignment
- Array of sensor frames (LiDAR ranges, odometry)

---

## Configuration

### Rover Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `module_id` | Required | Unique rover identifier |
| `record_rate_hz` | 10.0 | Sensor recording frequency |
| `linear_speed` | 0.4 | Forward velocity (m/s) |
| `angular_speed` | 0.6 | Rotation velocity (rad/s) |
| `obstacle_threshold` | 0.4 | Minimum obstacle distance (m) |
| `spawn_x/y/yaw` | 0.0 | Spawn position for teleport-back (sim only) |

### Dispatcher Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `coverage_goal` | 0.75 | Target map coverage (0-1) |
| `exploration_radius` | 4.0 | Max distance from dock (m) |
| `min_frontier_size` | 3 | Minimum frontier cell count |
| `auto_dispatch` | true | Automatically send idle rovers |

### SLAM Processor Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `playback_speed` | 10.0 | Replay speed multiplier |
| `scan_topic` | /offline_scan | Topic for replayed scans |
| `slam_output_dir` | "" | Directory for map output |

---

## Naming Convention

COVEN uses a witch/coven theme:

| Term | Meaning |
|------|---------|
| **Coven** | Central dock/base station (e.g., The_Graeae, The_Weird_Sisters) |
| **Witch** | Individual rover (e.g., Morrigan, Louhi, Baba_Yaga, Hecate) |
| **Mission** | Single exploration sortie |
| **Frontier** | Unexplored map boundary |

Names are drawn from mythology and pop culture.

---

## Project Structure

```
coven_core/
├── coven_core/
│   ├── data_mule_module.py      # Rover: navigate, record, return
│   ├── frontier_dispatcher.py   # Dock: analyze map, dispatch rovers
│   ├── offline_slam_processor.py # Dock: replay data through SLAM
│   └── common.py                # Shared utilities, naming
├── config/
│   └── slam_params_sim.yaml     # SLAM Toolbox configuration
├── launch/
│   └── coven_data_mule_sim.launch.py  # Main simulation launch
├── models/
│   ├── coven_rover/             # Rover SDF with LiDAR
│   └── dock.sdf                 # Dock model
├── worlds/
│   └── coven_4rover.sdf         # Simulation world
└── README.md
```

---

## Dependencies

```bash
# Core
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server

# Simulation
sudo apt install ros-jazzy-ros-gz ros-jazzy-robot-state-publisher
```

---

## Status

### Working
- Multi-rover dispatch in different directions (N, S, E, W, etc.)
- Sensor data recording during exploration (LiDAR + odometry)
- Obstacle avoidance and waypoint navigation
- Return-to-dock behavior
- Teleport back to spawn position (simulation)
- Mission queuing for overlapping returns
- State transition tracking (no race conditions)
- Session-based data organization
- Offline SLAM replay with world-frame transform

### Known Issues
- SLAM map saving may fail if `/map` topic isn't ready
- Coverage calculation requires proper TF chain

### Planned
- Physical hardware support
- Improved frontier selection (distance-based scoring)
- Multi-dock coordination
- Real wireless data transfer

---

## Research Context

COVEN addresses the "coordination gap" in planetary robotics—the absence of a standardized protocol enabling heterogeneous robots to work together without mission-specific pre-configuration.

### The Core Insight

The dock is **permanent infrastructure**—the reusable brain that outlives individual modules. Instead of building a new spacecraft for each mission:

1. **Land the dock once** - permanent coordination infrastructure
2. **Send modules incrementally** - cheaper, expendable, specialized
3. **Accumulate capability over time** - each mission adds tools to the fleet

The same dock that coordinates a single ReconRover in 2026 can coordinate a fleet of spectrometers, drills, and cargo haulers in 2036—without redesign.

### Key References
- CADRE (NASA JPL, 2024) - Swarm coordination
- HOTDOCK (DFKI, 2018) - Electromechanical docking
- Nav2 (Macenski et al., 2020) - Autonomous navigation
- Frontier Exploration (Yamauchi, 1997) - Autonomous mapping
- RoSE (Zhu et al., Colorado School of Mines) - Autonomous redocking

---

## Roadmap

### COVEN 1.x (Current)
- Data mule proof-of-concept with LiDAR
- Single-dock coordination
- Frontier-based exploration
- Offline SLAM processing

### COVEN 2.x
- **Specialized Module Types**
  - SpectrometerRover - Spectral analysis
  - DrillRover - Sample collection
  - CargoRover - Material transport
- Task-type matching based on capability

### COVEN 3.x
- **Multi-Dock Networks**
  - Dock-to-dock task handoff
  - Regional coverage zones
  - Distributed map sharing

### COVEN 4.x
- **Mini-Swarms**
  - Swarm leaders coordinate sub-teams
  - Formation-based exploration
  - Fault-tolerant redistribution

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

**ROS2 Jazzy | Python 3.12+ | Gazebo Harmonic**
