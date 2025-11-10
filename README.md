# COVEN

**Composable Operations for Versatile Exploration Networks**

A multi-robot coordination system for ROS2. Manages autonomous modules with dynamic task assignment, health monitoring, and hot-swap capability.

---

## What It Does

- **Dock-Module Architecture** - Central dock coordinates multiple robot modules
- **Automatic Discovery** - Modules register themselves on startup
- **Health Monitoring** - Heartbeat-based status tracking (1.25 Hz)
- **Task Assignment** - Dynamic mission distribution to available modules
- **State Machine** - BOOT → IDENTIFY → VERIFY → NORMAL → FIELD_OPS

Built for warehouse automation, exploration missions, and multi-robot research.

---

## Quick Start

### Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select coven_core --symlink-install
source install/setup.bash
```

### Run

Terminal 1 - Dock:
```bash
ros2 run coven_core dock
```

Terminal 2 - Module:
```bash
ros2 run coven_core module --ros-args -p skip_health_check:=true
```

Watch the discovery sequence in the logs. Module will identify itself and start sending heartbeats.

---

## Architecture

```
        ┌──────────────┐
        │     DOCK     │  ← Discovers modules, assigns tasks
        └──────┬───────┘
               │
        ROS2 /coven/* topics
               │
    ┌──────────┼──────────┐
    │          │          │
┌───▼────┐ ┌──▼─────┐ ┌──▼─────┐
│Module 1│ │Module 2│ │Module N│  ← Auto-register, report health, execute tasks
└────────┘ └────────┘ └────────┘
```

### State Flow

```
BOOT
  ↓ (receives IDENTIFY_REQ)
IDENTIFY
  ↓ (sends IDENTIFY_REP)
WAIT_VERIFY
  ↓ (receives VERIFY_REQ, responds VERIFY_REP)
  ↓ (receives POWER_ENABLE)
NORMAL
  ↓ (heartbeats at 1.25 Hz)
  ↓ (receives TASK_REQ)
FIELD_OPS
  ↓ (executes mission)
  ↓ (sends TASK_COMPLETE)
NORMAL
```

### Topics

**Discovery:**
- `/coven/identify_req` - Dock requests module IDs
- `/coven/identify_rep` - Modules respond with ID/type
- `/coven/verify_req` - Dock requests health check
- `/coven/verify_rep` - Modules report sensor status
- `/coven/enable_12v` - Dock enables module power

**Operations:**
- `/coven/heartbeat` - Module status (0.8s interval)
- `/coven/task_req` - Task assignment
- `/coven/task_ack` - Task acknowledgment
- `/coven/task_complete` - Task completion

All messages are JSON-encoded `std_msgs/String` for flexibility.

---

## Dependencies

### Core (Required)
```bash
sudo apt install ros-humble-desktop python3-pytest
```

### Optional (For autonomous navigation)
```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

---

## Testing

Run the test suite:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 -m pytest src/coven_core/test/ -v
```

Tests validate:
- Message encoding/decoding
- State machine transitions
- Discovery protocol
- Heartbeat timing
- Multi-module coordination

---

## Configuration

Key parameters (in module node):

```python
HB_PERIOD = 0.8          # Heartbeat interval (seconds)
HB_MISS_THRESHOLD = 3    # Heartbeats before timeout
IDENTIFY_INTERVAL = 5.0  # Discovery broadcast interval
```

To run with hardware health checks enabled, omit `skip_health_check`:
```bash
ros2 run coven_core module
```

Module will verify lidar (`/scan`) and navigation stack before going operational.

---

## Development

### Project Structure

```
coven_core/
├── coven_core/
│   ├── dock_node.py       # Dock coordination
│   ├── module_node.py     # Module FSM and task execution
│   ├── common.py          # Protocol definitions
│   └── exploration.py     # Autonomous navigation (optional)
└── test/
    ├── test_rigorous_integration.py   # Performance tests
    ├── test_protocol_validation.py    # Protocol tests
    ├── test_common.py                 # Message encoding tests
    └── test_fsm_transitions.py        # State machine tests
```

### Custom Modules

Extend `Module` class:

```python
from coven_core.module_node import Module

class CustomModule(Module):
    def __init__(self):
        super().__init__(module_id="CM-001", module_type="Custom")

    def execute_task(self, task_data):
        # Your mission logic here
        pass
```

---

## Status

**Production-Ready:**
- ✅ Multi-module coordination (tested with 5 concurrent modules)
- ✅ Discovery and registration (<5ms latency)
- ✅ Heartbeat monitoring (1.23 Hz measured)
- ✅ Task assignment protocol
- ✅ State machine transitions

**In Development:**
- Navigation integration (Nav2 issues being resolved)
- Hardware docking mechanism

---

## License

MIT License - See LICENSE file

---

## Author

Alexander Shultis
University of Hawaiʻi at Mānoa
Department of Astronomy
November 2025

---

**ROS2 Humble | Python 3.10+**
