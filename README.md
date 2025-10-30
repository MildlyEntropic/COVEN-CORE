# COVEN-CORE

**COVEN** (Composable Operations for Versatile Exploration Networks) is a ROS2-based modular robotics framework enabling autonomous dock-and-rover swarms with hot-pluggable modules and standardized interfaces.

**Phase 1** validates plug-level lifecycle logic (*connect, verify, operate, return*) in simulation before hardware fabrication.

---

## ⚡ Quick Start

### Ultra-Simple (5 seconds)

```bash
cd ~/ros2_ws
./coven
```

That's it! COVEN dock + module running, no dependencies.

### With Exploration

```bash
cd ~/ros2_ws
./coven full
```

Wait 15 seconds, then send a mission:
```bash
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```

Watch the robot explore, build a map, return to dock, and transfer data!

---

## 🎯 What COVEN Does

### Core Features
- **Hot-dockable modules** - Connect/disconnect at runtime
- **FSM-based lifecycle** - BOOT → IDENTIFY → VERIFY → NORMAL → FIELD_OPS
- **Heartbeat monitoring** - 3-miss threshold with recovery
- **Multi-module support** - Dock manages multiple rovers concurrently
- **Autonomous exploration** - Frontier-based navigation with SLAM
- **Map data transfer** - Compressed occupancy grids to dock
- **Hardware-agnostic** - Works with any robot providing `/cmd_vel`, `/scan`, `/odom`

### Use Cases
- Research swarm coordination
- Modular robot development
- Autonomous exploration
- Multi-agent task allocation
- Hot-swappable sensor platforms

---

## 🚀 Launch Modes

```bash
./coven           # Basic: dock + module only
./coven sim       # Add simulation
./coven nav       # Add navigation
./coven full      # Everything (sim + nav + COVEN)
./coven help      # Show usage
```

### Basic Mode (Default)
- No simulation
- No navigation
- Just COVEN FSM
- **Perfect for:** Testing communication, developing behaviors

### Sim Mode
- Create3 Gazebo simulation
- COVEN dock + module
- **Perfect for:** Testing with robot dynamics

### Nav Mode
- SLAM + Nav2 stack
- COVEN dock + module
- **Perfect for:** Real hardware testing

### Full Mode
- Simulation + Navigation + COVEN
- **Perfect for:** Complete exploration missions

---

## 📊 System Architecture

```
┌─────────────────────────────────────────┐
│  OPERATOR                               │
│  ./coven full                           │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│  COVEN CORE (always included)           │
│  ┌────────────┐     ┌─────────────────┐ │
│  │ DockMulti  │◄────┤ Module(s)       │ │
│  │ Hub        │     │ - FSM lifecycle  │ │
│  │            │     │ - Heartbeat      │ │
│  │            │     │ - Task execution │ │
│  └────────────┘     └─────────────────┘ │
└─────────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│  NAVIGATION (optional - with_nav)       │
│  - SLAM Toolbox (mapping)               │
│  - Nav2 (planning/control)              │
│  - Frontier exploration                 │
└─────────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│  SIMULATION (optional - with_sim)       │
│  - Create3 Gazebo                       │
│  - Physics engine                       │
│  - Sensor simulation                    │
└─────────────────────────────────────────┘
```

---

## 📡 Communication Protocol

### COVEN Topics

**Connection Lifecycle:**
- `/coven/identify_req` - Dock broadcasts discovery (5s interval)
- `/coven/identify_rep` - Module responds with ID/type/firmware
- `/coven/verify_req` - Dock requests verification
- `/coven/verify_rep` - Module confirms verification
- `/coven/enable_12v` - Dock enables module power

**Runtime:**
- `/coven/heartbeat` - Module health (0.8s interval)

**Task Management:**
- `/coven/mission_req` - High-level mission commands
- `/coven/task_req` - Task assignment to specific module
- `/coven/task_ack` - Module accepts/rejects task
- `/coven/task_start` - Module begins field operations
- `/coven/task_complete` - Module completes (includes map data)

### Message Format

All messages use JSON-encoded `std_msgs/String` for flexibility:

```json
{
  "module_id": "RR-a3b4c5",
  "task": "explore_warehouse",
  "success": true,
  "map_data": "<base64-encoded-gzipped-pgm>",
  "map_yaml": "<base64-encoded-gzipped-yaml>",
  "exploration_metrics": {
    "duration": 247.3,
    "coverage": 0.785,
    "iterations": 12,
    "distance_traveled": 45.2
  }
}
```

---

## 🗺️ Exploration Missions

### Send a Mission

```bash
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```

### What Happens

1. Dock assigns task to available module
2. Module initializes Nav2 + SLAM
3. Module stores dock position
4. Module explores using frontier-based navigation
5. SLAM builds occupancy grid map
6. Module returns to dock coordinates
7. Module saves and compresses map
8. Module transmits map data to dock
9. Dock stores map files + metrics

### View Results

```bash
ls -R ~/coven_maps/
cat ~/coven_maps/*/*/metrics.json | jq
```

**Map Storage:**
```
~/coven_maps/
└── RR-a3b4c5/
    └── explore_warehouse_20251029_143022/
        ├── exploration_map.pgm    # Occupancy grid
        ├── exploration_map.yaml   # Metadata
        └── metrics.json           # Statistics
```

---

## 🛠️ Dependencies

### Core COVEN (minimal)
```bash
sudo apt install ros-jazzy-rclpy ros-jazzy-std-msgs
```

### For Exploration (optional)
```bash
sudo apt install \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-slam-toolbox

pip install numpy scipy
```

### For Simulation (optional)
```bash
sudo apt install \
  ros-jazzy-irobot-create-gz-bringup \
  ros-jazzy-ros-gz-bridge
```

---

## 📖 Documentation

- **[LAUNCHER.md](LAUNCHER.md)** - Detailed launcher usage
- **[SIMPLE_APPROACH.md](SIMPLE_APPROACH.md)** - Architecture philosophy
- **[EXPLORATION.md](EXPLORATION.md)** - Exploration system guide
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues
- **[README_DEV.md](README_DEV.md)** - Developer guide

---

## 🧪 Testing

### Level 1: Basic FSM
```bash
./coven
# Send test mission
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```
**Validates:** Connection, heartbeat, task execution

### Level 2: With Simulation
```bash
./coven sim
```
**Validates:** Robot integration, sensor data

### Level 3: Full Exploration
```bash
./coven full
# Wait 15s, then:
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```
**Validates:** Navigation, SLAM, map transfer

---

## 🔧 Configuration

### Navigation Parameters
**File:** `config/nav2_simple.yaml`

Key settings:
- Robot radius: 0.22m
- Max velocity: 0.5 m/s
- Costmap resolution: 0.05m

### SLAM Parameters
**File:** `config/slam_simple.yaml`

Key settings:
- Map resolution: 0.05m
- Update interval: 1.0s
- Loop closure: enabled

### Exploration Parameters
**File:** `coven_core/exploration.py`

```python
EXPLORATION_TIMEOUT = 300.0    # seconds
COVERAGE_THRESHOLD = 0.80      # 80%
FRONTIER_SEARCH_RADIUS = 3.0   # meters
MIN_FRONTIER_SIZE = 10         # cells
```

---

## 🏗️ Building

```bash
cd ~/ros2_ws
colcon build --packages-select coven_core --symlink-install
source install/setup.bash
```

---

## 🎓 Module Development

### Create a Custom Module

```python
from coven_core.module_node import Module

class MyModule(Module):
    def __init__(self):
        super().__init__(
            module_id="MM-custom",
            module_type="MyModule",
            fw="1.0.0"
        )

    def execute_task(self, task_name):
        # Your custom behavior
        pass
```

### Add New Mission Types

Edit `module_node.py`:

```python
if "explore" in task_name.lower():
    # Exploration behavior
elif "patrol" in task_name.lower():
    # Your patrol behavior
elif "survey" in task_name.lower():
    # Your survey behavior
```

---

## 🔬 Research & Hardware

### Phase 1 (Current): Simulation Validation
- ✅ FSM lifecycle logic
- ✅ Multi-module coordination
- ✅ Autonomous exploration
- ✅ Map data transfer

### Phase 2 (Future): Hardware Integration
- 🔄 CubeRover fabrication (100×100×50mm)
- 🔄 9-pin hot-dock connector
- 🔄 IR beacon auto-docking
- 🔄 Multi-rover field testing

---

## 📊 Performance

**Typical Warehouse Exploration (100m²):**
- Duration: 3-5 minutes
- Coverage: 70-85%
- Iterations: 8-15 waypoints
- Map size: 200-400 KB (compressed)
- Distance: 40-60 meters

---

## 🤝 Contributing

COVEN is research code from University of Hawaii / Colorado School of Mines.

**Maintainer:** Alexander Shultis (shultis@hawaii.edu)

---

## 📜 License

MIT License - See LICENSE file

---

## 🙏 Acknowledgments

- **ROS2** - Robot Operating System
- **Nav2** - Navigation framework
- **SLAM Toolbox** - Mapping system
- **iRobot Create3** - Robot platform base

---

**Built with:** ROS2 Jazzy | Python 3 | Nav2 | SLAM Toolbox
**Status:** Phase 1 - Simulation Validated ✅
**Next:** Hardware Fabrication & Field Testing
