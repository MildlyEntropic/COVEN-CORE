# COVEN Universal Launcher

## One Launch File, All Modes

We consolidated **4 old launch files** into **1 flexible launcher** that handles everything.

---

## Quick Start

### Using the Shell Script (Easiest)

```bash
# Basic mode (no sim, no nav - just COVEN)
~/ros2_ws/coven

# With simulation
~/ros2_ws/coven sim

# With navigation (requires robot/sim)
~/ros2_ws/coven nav

# Full stack (sim + nav + COVEN)
~/ros2_ws/coven full
```

### Using ROS Launch Directly

```bash
# Basic: dock + module only
ros2 launch coven_core coven.launch.py

# With simulation
ros2 launch coven_core coven.launch.py with_sim:=true

# With navigation
ros2 launch coven_core coven.launch.py with_nav:=true

# Full stack
ros2 launch coven_core coven.launch.py with_sim:=true with_nav:=true

# Custom world
ros2 launch coven_core coven.launch.py with_sim:=true world:=maze
```

---

## Launch Modes

### **Basic Mode** (Default)
```bash
~/ros2_ws/coven
```

**Launches:**
- ✅ COVEN Dock
- ✅ COVEN Module

**Use for:**
- Testing FSM logic
- Developing dock/module communication
- Running without simulation/navigation

**Test with:**
```bash
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

**Expected:** Module connects, shows heartbeat, executes 5s task, completes

---

### **Sim Mode**
```bash
~/ros2_ws/coven sim
```

**Launches:**
- ✅ Create3 Gazebo simulation
- ✅ COVEN Dock
- ✅ COVEN Module

**Use for:**
- Testing with simulated robot
- Manual teleop (use keyboard_teleop or joystick)
- Verifying robot behaviors

**Topics available:**
- `/cmd_vel` - Control robot
- `/scan` - LiDAR data
- `/odom` - Odometry

---

### **Nav Mode**
```bash
~/ros2_ws/coven nav
```

**Launches:**
- ✅ SLAM Toolbox
- ✅ Nav2 stack (planner, controller, behaviors)
- ✅ COVEN Dock
- ✅ COVEN Module

**Use for:**
- Testing navigation with real robot
- Running on hardware (provide /cmd_vel, /scan, /odom)

**Prerequisites:**
- Robot (real or sim) must be running first
- Must have /cmd_vel, /scan, /odom topics

---

### **Full Mode** (Exploration Ready)
```bash
~/ros2_ws/coven full
# or
~/ros2_ws/coven explore
```

**Launches:**
- ✅ Create3 Gazebo simulation
- ✅ SLAM Toolbox
- ✅ Nav2 stack
- ✅ COVEN Dock
- ✅ COVEN Module

**Use for:**
- Complete exploration missions
- Integration testing
- Demos

**Send exploration mission:**
```bash
# Wait 15 seconds after launch
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```

**What happens:**
1. Module accepts task
2. Initializes Nav2 + SLAM
3. Explores autonomously
4. Builds map
5. Returns to dock
6. Transfers map to dock
7. Map saved to `~/coven_maps/`

---

## Launch Arguments

### **with_sim** (default: false)
Launch Create3 Gazebo simulation

```bash
ros2 launch coven_core coven.launch.py with_sim:=true
```

### **with_nav** (default: false)
Launch navigation stack (SLAM + Nav2)

```bash
ros2 launch coven_core coven.launch.py with_nav:=true
```

### **world** (default: warehouse)
Gazebo world to load (only with `with_sim:=true`)

```bash
ros2 launch coven_core coven.launch.py with_sim:=true world:=maze
```

**Available worlds:**
- `warehouse` - Large indoor space (default)
- `maze` - Complex navigation environment
- `depot` - Structured storage area
- `empty` - Blank world for testing

### **use_sim_time** (default: true)
Use simulation time instead of wall clock

```bash
ros2 launch coven_core coven.launch.py use_sim_time:=false
```

---

## Architecture

```
coven.launch.py
├── Arguments (with_sim, with_nav, world, use_sim_time)
│
├── [Simulation] (if with_sim=true)
│   └── Create3 Gazebo + world
│
├── [Navigation] (if with_nav=true)
│   ├── SLAM Toolbox
│   ├── Nav2 Controller
│   ├── Nav2 Planner
│   ├── Nav2 Behaviors
│   ├── Nav2 BT Navigator
│   ├── Nav2 Waypoint Follower
│   ├── Velocity Smoother
│   └── Lifecycle Manager
│
└── [COVEN] (always)
    ├── Dock (delayed 5s)
    └── Module (delayed 10s)
```

---

## Timing

The launcher uses delays to ensure proper startup order:

- **T+0s:** Simulation/navigation nodes start
- **T+5s:** Dock launches
- **T+10s:** Module launches

This prevents race conditions where module tries to connect before dock is ready.

---

## File Cleanup

### Old Launch Files (Removed)
- ❌ `dock_and_module.launch.py` - Replaced by `coven.launch.py`
- ❌ `exploration_demo.launch.py` - Replaced by `coven.launch.py with_sim:=true with_nav:=true`
- ❌ `module_with_nav.launch.py` - Replaced by `coven.launch.py with_nav:=true`
- ❌ `coven_explore_simple.launch.py` - Replaced by `coven.launch.py`

### New Launch File
- ✅ `coven.launch.py` - Handles all modes

**Result:** 4 files → 1 file (250 lines → 200 lines)

---

## Testing Progression

### Level 1: Basic COVEN
```bash
~/ros2_ws/coven
```
**Validates:** FSM, communication, heartbeat

### Level 2: With Simulation
```bash
~/ros2_ws/coven sim
```
**Validates:** Robot integration, sensor data

### Level 3: With Navigation
```bash
~/ros2_ws/coven full
```
**Validates:** Path planning, SLAM, exploration

Start simple, add complexity!

---

## Troubleshooting

### "No executable found"

**Cause:** Haven't built package

**Fix:**
```bash
cd ~/ros2_ws
colcon build --packages-select coven_core --symlink-install
source install/setup.bash
```

### "with_sim:=true fails"

**Cause:** irobot_create_gz_bringup not installed

**Fix:** Don't use sim mode, or install:
```bash
sudo apt install ros-jazzy-irobot-create-gz-bringup
```

### "Navigation nodes crash"

**Cause:** No robot topics (/cmd_vel, /scan, /odom)

**Fix:** Launch simulation first:
```bash
~/ros2_ws/coven sim    # Don't use 'nav' mode alone
```

### "Module doesn't explore"

**Causes:**
1. Nav2 not initialized (wait 15s)
2. No frontiers detected (map too small)
3. Task name doesn't contain "explore"

**Debug:**
```bash
ros2 node list | grep navigation
ros2 topic hz /map
```

---

## Examples

### Basic COVEN Testing
```bash
# Terminal 1
~/ros2_ws/coven

# Terminal 2 (after 10s)
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

### Exploration Mission
```bash
# Terminal 1
~/ros2_ws/coven full

# Terminal 2 (after 15s)
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'

# Terminal 3 (watch progress)
ros2 topic echo /coven/heartbeat        # Module health
ros2 topic echo /coven/task_complete    # Mission result

# Terminal 4 (view map)
ros2 topic hz /map                      # SLAM update rate
```

### Check Results
```bash
ls -R ~/coven_maps/
cat ~/coven_maps/*/*/metrics.json | jq
```

---

## Help

```bash
~/ros2_ws/coven help
```

Shows usage and examples.

---

**Author:** Alexander Shultis
**Date:** October 2025
**Philosophy:** One launcher to rule them all.
