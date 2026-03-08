# COVEN

**Composable Object-oriented Versatile Emergent Networks**

A multi-robot exploration system using the **data mule** pattern for efficient SLAM in resource-constrained environments. Rovers run Rust firmware on Raspberry Pi Zero 2W; the dock runs Python/ROS2 on a Raspberry Pi 4.

---

## What It Does

COVEN deploys multiple lightweight rovers ("witches") from a central dock ("coven") to explore unknown environments. Rovers are simple data collectors—they record sensor data during exploration and physically return to the dock for centralized SLAM processing via wired USB connection.

```
                    ┌─────────────────┐
                    │     DOCK        │
                    │  (The Coven)    │
                    │  Pi 4 / ROS2   │
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
                             │ USB/UART
            ┌────────────────┼────────────────┐
            │                │                │
            ▼                ▼                ▼
       ┌─────────┐      ┌─────────┐      ┌─────────┐
       │ Witch 1 │      │ Witch 2 │      │ Witch N │
       │ (Rover) │      │ (Rover) │      │ (Rover) │
       │ Pi Zero │      │ Pi Zero │      │ Pi Zero │
       │  Rust   │      │  Rust   │      │  Rust   │
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
- Returns to dock for centralized processing via wired connection
- Enables offline SLAM with full computational resources

This makes COVEN ideal for:
- Swarm robotics with many low-cost units
- Communication-denied environments (caves, lunar pits)
- Scenarios where map quality matters more than real-time updates

---

## Architecture

### Two-Stack Design

| Component | Platform | Language | Purpose |
|-----------|----------|----------|---------|
| Rover firmware | Pi Zero 2W | Rust | Navigate, record sensors, return data |
| Dock software | Pi 4 | Python/ROS2 | Coordinate rovers, run offline SLAM |

Communication between dock and rover uses a binary UART protocol with COBS framing over USB.

### Rover State Machine

```
Disconnected → Identifying → Normal → FieldOps → Docking → Uploading → Normal
                                          │                                │
                                          └────────────────────────────────┘
```

### Subsumption Layers (Rover Navigation)

| Priority | Layer | Role |
|----------|-------|------|
| L0 | Obstacle Avoidance | LiDAR repulsive field — always active |
| L1 | Return to Dock | Default fallback — low battery / no task |
| L2 | Wander & Collect | Opportunistic data — scenic return |
| L3 | Navigate to Goal | Potential field to dock-assigned frontier |
| L4 | Execute Task | Polymorphic — varies by rover type |

### Data Flow

1. **Connect**: Rover docks, USB UART handshake establishes identity
2. **Dispatch**: Dock sends frontier waypoint to idle rover
3. **Explore**: Rover navigates toward target, recording LiDAR + odometry
4. **Return**: Rover heads back to dock when target reached or battery low
5. **Upload**: Rover transfers binary sensor batch over UART
6. **Process**: Offline SLAM replays data, updates occupancy grid map
7. **Repeat**: Frontier analysis finds new targets, cycle continues

---

## Quick Start

### Dock (Raspberry Pi 4)

```bash
# Build dock software in Docker
cd coven_core/docker
docker build -t coven-ros2 .

# Launch dock coordination
ros2 launch coven_core coven_dock_hardware.launch.py
```

### Rover (Raspberry Pi Zero 2W)

```bash
# Build rover firmware
cd coven_core/rover
cargo build --release

# Run with hardware
./target/release/coven-rover

# Run in mock mode (no hardware)
./target/release/coven-rover --mock
```

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

---

## Configuration

### Rover (`rover.toml`)

| Section | Key Parameters |
|---------|----------------|
| `[hardware.motors]` | PWM pins, direction pins, frequency |
| `[hardware.encoders]` | Encoder pins, ticks per revolution |
| `[hardware.lidar]` | Serial port, baud rate |
| `[navigation]` | d_safe, max speeds, Lyapunov gains |
| `[battery]` | ADC address, voltage divider ratio |

### Dock (Launch Arguments)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `coven_name` | The_Graeae | Dock identifier |
| `dock_x/y` | 0.0 | Dock position in map frame |
| `data_dir` | ~/Desktop/COVEN/Data | Data storage directory |

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
├── rover/                          # Rust rover firmware
│   ├── src/
│   │   ├── main.rs                 # Entry point, mock mode
│   │   ├── state.rs                # Rover state machine
│   │   ├── navigation.rs           # Lyapunov nav, waypoint follower
│   │   ├── subsumption.rs          # L0-L4 behavioral layers
│   │   ├── dock_uart.rs            # Binary UART protocol
│   │   ├── protocol.rs             # Message type definitions
│   │   └── hardware/               # Motor, encoder, LiDAR, battery drivers
│   ├── rover.toml                  # Rover configuration
│   └── Cargo.toml
├── coven_core/                     # Python dock software
│   ├── rover_bridge.py             # UART bridge: rovers ↔ ROS2
│   ├── bridge_protocol.py          # COVEN handshake protocol
│   ├── bridge_data.py              # Batch processing and disk persistence
│   ├── frame_codec.py              # COBS framing and message encoding
│   ├── frontier_dispatcher.py      # Frontier detection and rover dispatch
│   ├── frontier_analysis.py        # Frontier clustering algorithms
│   ├── offline_slam_processor.py   # Replay sensor data through SLAM
│   └── task_auctioneer.py          # Mission assignment and tracking
├── config/
│   └── slam_params_hw.yaml         # SLAM Toolbox configuration
├── launch/
│   └── coven_dock_hardware.launch.py
├── docker/
│   └── Dockerfile                  # ROS2 Jazzy development container
└── README.md
```

---

## Dependencies

### Rover
- Rust toolchain (stable)
- `rppal` (GPIO/I2C/SPI for Raspberry Pi)
- `serialport` (LiDAR UART)

### Dock
```bash
# Core ROS2 packages
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server

# Python
pip install pyserial numpy
```

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

## License

MIT License - See LICENSE file

---

## Author

Alexander Shultis
University of Hawaiʻi at Mānoa
Department of Astronomy

Faculty Advisor: Dr. Miguel Nunes (HSFL)
Thesis Advisor: Dr. Jiaoyang Zhu (Colorado School of Mines)

---

**Rust | ROS2 Jazzy | Python 3.12+ | Raspberry Pi**
