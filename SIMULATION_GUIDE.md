# COVEN Simulation Guide

All simulation modes now preserve beautiful real-time color-coded logs by running COVEN nodes directly with `ros2 run` instead of through launch files!

## Quick Start

### Basic Simulation (No Navigation)
```bash
coven sim
```

**What happens**:
1. Gazebo launches with Create3 robot (~15s)
2. Dock starts with real-time logs
3. Module starts with real-time logs

**Use for**: Testing COVEN FSM with robot visualization

### Simulation + Navigation
```bash
coven full
```

**What happens**:
1. Gazebo launches (~15s)
2. SLAM Toolbox starts
3. Nav2 starts
4. Dock starts with real-time logs
5. Module starts with real-time logs

**Use for**: Full exploration missions with mapping

### Navigation Only (Bring Your Own Robot/Sim)
```bash
# Terminal 1 - Start your robot or sim manually
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# Terminal 2 - Start COVEN with navigation
coven nav
```

**Use for**: Testing with hardware or custom simulation

## What You'll See

### Beautiful Logs Preserved! 🎨

```
================================================
  COVEN Simulation Mode
================================================

Starting Gazebo simulation...
(This will take ~15 seconds to load)

Gazebo started (PID: 12345)
Waiting for simulation to initialize...

Starting COVEN nodes...
Started dock (PID: 12346)

[DOCK] 🎯 COVEN Multi-Module Dock v2.0
[DOCK] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[DOCK] 📡 ROS2 Node: coven_dock
[DOCK] 🏢 Dock ID: DD-a1b2c3
[DOCK] 📋 Max Capacity: 10 modules
[DOCK] 💾 Map Storage: ~/coven_maps/
[DOCK] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[DOCK] 🔄 FSM: BOOT
[DOCK] ⏳ Waiting 3.0s before IDENTIFY...

Started module (PID: 12347)

[MODULE] 🤖 COVEN Roaming Module v2.0
[MODULE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━
[MODULE] 📡 ROS2 Node: coven_module
[MODULE] 🆔 Module ID: RR-x7y8z9
[MODULE] 🔋 Battery: 100%
[MODULE] 🗺️  Map Storage: ~/coven_maps/
[MODULE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━

All nodes started!
  Gazebo PID: 12345
  Dock PID: 12346
  Module PID: 12347

Open Gazebo to see the robot: http://localhost:8080
Press Ctrl+C to stop
```

All the beautiful color-coded output streams in real-time!

## Sending Test Missions

### Simple Test Mission
```bash
# In another terminal
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

### Exploration Mission (full mode only)
```bash
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```

## Monitoring

### Watch All Activity
The logs stream directly to your terminal in real-time with colors!

### Additional Monitoring (Optional)
```bash
# Terminal 2 - Watch heartbeats
ros2 topic echo /coven/heartbeat

# Terminal 3 - Watch task completions
ros2 topic echo /coven/task_complete

# Terminal 4 - Monitor navigation goals (full mode)
ros2 topic echo /goal_pose
```

## Architecture

### Sim Mode Process Tree
```
coven sim
├── Gazebo (launch file, buffered logs)
├── Dock (ros2 run, real-time logs) ✓
└── Module (ros2 run, real-time logs) ✓
```

### Full Mode Process Tree
```
coven full
├── Gazebo (launch file, buffered logs)
├── SLAM Toolbox (launch file, buffered logs)
├── Nav2 (launch file, buffered logs)
├── Dock (ros2 run, real-time logs) ✓
└── Module (ros2 run, real-time logs) ✓
```

**Key**: Navigation infrastructure runs via launch files (necessary for complex configs), but COVEN nodes run directly to preserve beautiful logs!

## Signal Handling

Press **Ctrl+C** once to cleanly stop all processes:
- Gazebo
- SLAM (if running)
- Nav2 (if running)
- Dock
- Module(s)

All PIDs are tracked and killed together.

## Troubleshooting

### "Gazebo not found"
```bash
# Install TurtleBot4 simulator
sudo apt install ros-jazzy-turtlebot4-simulator
```

### "SLAM Toolbox not found"
```bash
sudo apt install ros-jazzy-slam-toolbox
```

### "Nav2 not found"
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### Gazebo Opens Blank
- Wait the full 15 seconds for initialization
- Check Gazebo GUI: http://localhost:8080
- If still blank, restart: `pkill -9 gz` then `coven sim`

### No Logs Appearing
- The new system shows logs in real-time!
- If you see buffered/aggregated logs, you might be using the old launch file
- Make sure you're using `coven sim` or `./coven sim`, not `ros2 launch`

### Navigation Not Working
- Make sure you used `coven full`, not `coven sim`
- Wait 30 seconds total for all nodes to stabilize
- Check topics: `ros2 topic list | grep -E "(scan|cmd_vel|odom|map)"`

### Simulation Time Issues
All nodes automatically use `use_sim_time:=true` in sim/nav/full modes. The launcher now explicitly waits for Gazebo's `/clock` topic to be publishing before starting COVEN nodes, ensuring proper time synchronization. No manual configuration needed!

## Performance Notes

### Startup Times
- **Gazebo**: ~20 seconds (includes clock initialization)
- **SLAM + Nav2**: ~18 seconds
- **COVEN nodes**: ~15 seconds (dock initialization)
- **Total (sim mode)**: ~35 seconds
- **Total (full mode)**: ~55 seconds

### Resource Usage
- **Gazebo**: ~2GB RAM, 1-2 CPU cores
- **Nav2 stack**: ~500MB RAM, 1 CPU core
- **COVEN nodes**: ~50MB RAM, minimal CPU

### Optimization Tips
```bash
# Reduce Gazebo resource usage
export GZ_SIM_RESOURCE_PATH=
export GZ_SIM_SYSTEM_PLUGIN_PATH=

# Use headless mode (no GUI)
export HEADLESS=1
coven sim
```

## Comparison: Old vs New

### Old Way (Buffered Logs)
```bash
coven sim
# All logs mixed together, buffered, hard to read
# Nav2 spam drowns out COVEN logs
```

### New Way (Real-Time Logs)
```bash
coven sim
# COVEN logs stream in real-time with colors!
# Nav2 runs in background
# Clean, readable, beautiful
```

## Next Steps

1. **Test sim mode**: `coven sim` - see robot in Gazebo
2. **Test mission**: Send test mission and watch execution
3. **Test full mode**: `coven full` - autonomous exploration
4. **Customize**: Edit [module_node.py](coven_core/module_node.py) for custom behaviors

---

**TL;DR**: Use `coven sim` for basic testing with Gazebo, `coven full` for complete exploration missions. Both preserve beautiful real-time logs while keeping Nav2 infrastructure in the background!
