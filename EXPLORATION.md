# COVEN Exploration Integration Guide

**Phase 1 Navigation Integration — October 2025**

This document describes the autonomous exploration capabilities added to COVEN modules for LiDAR-equipped CubeRover exploration and mapping missions.

---

## Overview

COVEN modules can now perform autonomous exploration missions using:
- **Nav2** navigation stack for path planning
- **SLAM Toolbox** for real-time map building
- **Frontier-based exploration** to systematically discover unknown areas
- **Automatic dock return** after mission completion
- **Map data transfer** from module to dock via existing COVEN protocol

---

## Architecture

```
Mission Flow:
1. Dock receives mission request (e.g., "explore_warehouse")
2. Dock assigns task to available module
3. Module initializes Nav2 + SLAM
4. Module stores initial dock position
5. Module explores using frontier-based navigation
6. Module builds occupancy grid map via SLAM
7. Module returns to dock coordinates
8. Module saves and serializes map data
9. Module transmits map + metrics to dock
10. Dock stores map files in ~/coven_maps/{module_id}/
```

---

## Quick Start

### Option 1: Full Demo Launch (Recommended)

Launch everything with a single command:

```bash
ros2 launch coven_core exploration_demo.launch.py
```

This starts:
- TurtleBot4 Gazebo simulation (warehouse world)
- SLAM Toolbox
- Nav2 navigation stack
- COVEN dock node
- COVEN module node

Wait ~15 seconds for all nodes to initialize, then send an exploration mission:

```bash
ros2 topic pub --once /coven/mission_req std_msgs/msg/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```

Or use the existing mission script:

```bash
~/bin/coven.mission explore_warehouse
```

### Option 2: Manual Launch (For Debugging)

**Terminal 1 - Simulation:**
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=warehouse
```

**Terminal 2 - SLAM:**
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

**Terminal 3 - Nav2:**
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

**Terminal 4 - Dock:**
```bash
ros2 run coven_core dock_multi
```

**Terminal 5 - Module:**
```bash
ros2 run coven_core module
```

**Terminal 6 - Mission Command:**
```bash
ros2 topic pub --once /coven/mission_req std_msgs/msg/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'
```

---

## Exploration Behavior

### Frontier Detection

The explorer uses frontier-based navigation:
- Frontiers are boundaries between known free space and unknown space
- Uses occupancy grid from SLAM to detect frontiers
- Requires `numpy` and `scipy` for image processing
- Falls back to random waypoint generation if frontier detection fails

### Termination Criteria

Exploration ends when ANY of these conditions is met:
- **Time limit reached** (default: 300 seconds / 5 minutes)
- **Coverage threshold** (default: 80% of reachable area)
- **No new frontiers** (3 consecutive iterations without frontiers)

Adjust in `exploration.py`:
```python
EXPLORATION_TIMEOUT = 300.0  # seconds
COVERAGE_THRESHOLD = 0.80    # 80%
NO_FRONTIER_LIMIT = 3        # iterations
```

### Return to Dock

After exploration:
1. Module navigates back to stored dock coordinates
2. Uses Nav2's `goToPose()` for autonomous return
3. If return fails, module uses best-effort positioning

---

## Map Data Storage

### On Module

Maps are temporarily saved to `/tmp/coven_maps/{module_id}/`:
- `exploration_map.pgm` - Grayscale occupancy grid image
- `exploration_map.yaml` - Map metadata (resolution, origin, thresholds)

### On Dock

Maps are stored in `~/coven_maps/{module_id}/{task}_{timestamp}/`:
- `exploration_map.pgm` - Reconstructed map image
- `exploration_map.yaml` - Map metadata
- `metrics.json` - Exploration statistics

Example:
```
~/coven_maps/
└── RR-a3b4c5/
    └── explore_warehouse_20251029_143022/
        ├── exploration_map.pgm
        ├── exploration_map.yaml
        └── metrics.json
```

### Metrics JSON Example

```json
{
  "module_id": "RR-a3b4c5",
  "task": "explore_warehouse",
  "timestamp": "20251029_143022",
  "success": true,
  "note": "Exploration complete: 78.5% coverage",
  "metrics": {
    "duration": 247.3,
    "iterations": 12,
    "coverage": 0.785,
    "explored_cells": 15678,
    "distance_traveled": 45.2
  }
}
```

---

## Message Protocol Extensions

### TaskComplete Message

Extended to include map data:

```python
@dataclass
class TaskComplete:
    module_id: str
    task: str
    success: bool = True
    note: str = ""
    map_data: str = ""           # Base64-encoded gzipped PGM
    map_yaml: str = ""           # Base64-encoded gzipped YAML
    exploration_metrics: dict = {}
```

### Exploration Metrics

```python
{
    "duration": float,           # seconds
    "iterations": int,           # exploration cycles
    "coverage": float,           # 0.0 to 1.0
    "explored_cells": int,       # occupancy grid cells
    "distance_traveled": float   # meters (estimated)
}
```

---

## Configuration

### Exploration Parameters

Edit `coven_core/coven_core/exploration.py`:

```python
FRONTIER_SEARCH_RADIUS = 3.0   # meters
MIN_FRONTIER_SIZE = 10         # cells
EXPLORATION_TIMEOUT = 300.0    # seconds
COVERAGE_THRESHOLD = 0.80      # 0.0 to 1.0
NO_FRONTIER_LIMIT = 3          # iterations
```

### SLAM Parameters

Default SLAM config from TurtleBot4:
- `/home/ander/ros2_ws/src/turtlebot4/turtlebot4_navigation/config/slam.yaml`

Key settings:
- Resolution: 0.05m (5cm per cell)
- Update rate: 5Hz
- Mode: Online synchronous

### Nav2 Parameters

Default Nav2 config from TurtleBot4:
- `/home/ander/ros2_ws/src/turtlebot4/turtlebot4_navigation/config/nav2.yaml`

Uses:
- Global planner: NavFn (Dijkstra)
- Local planner: DWB (Dynamic Window Approach)
- Recovery behaviors: Clear costmap, spin, backup

---

## Troubleshooting

### Module doesn't start exploration

**Symptoms:** Module accepts task but immediately completes without moving.

**Causes:**
1. Nav2 not ready - check `ros2 node list` includes navigation nodes
2. SLAM not publishing map - check `ros2 topic echo /map`
3. Initial pose not set - SLAM needs localization

**Solutions:**
- Wait 10-15 seconds after launch before sending mission
- Verify Nav2 active: `ros2 topic list | grep navigation`
- Check logs: `ros2 run coven_core module` (look for "Navigation components ready")

### Frontier detection not working

**Symptoms:** Log shows "scipy not available" or "No significant frontiers found"

**Causes:**
1. Missing `scipy` dependency
2. Map not yet built by SLAM
3. Already explored (no unknown areas)

**Solutions:**
- Install scipy: `pip install scipy`
- Wait for SLAM to build initial map (~30 seconds)
- Use larger environment or restart SLAM

### Map files not saved

**Symptoms:** TaskComplete shows success but no map files in `~/coven_maps/`

**Causes:**
1. `nav2_map_server` not available
2. Map saving timeout
3. Permission issues

**Solutions:**
- Verify package: `ros2 pkg list | grep nav2_map_server`
- Check module logs for "Map saver failed"
- Ensure write permissions: `ls -la ~/coven_maps/`

### Module doesn't return to dock

**Symptoms:** Module completes exploration but stops elsewhere

**Causes:**
1. Dock pose not stored correctly
2. Navigation goal unreachable
3. Nav2 timeout or failure

**Solutions:**
- Check initial pose logic in `_get_current_pose()`
- Verify dock coordinates accessible (not blocked)
- Increase Nav2 timeout in config

---

## Advanced Usage

### Custom Exploration Mission

Create mission-specific behaviors by checking task name:

```python
# In module_node.py execute_task()
if "explore_warehouse" in task_name.lower():
    success, metrics = self.explorer.explore(duration=300)
elif "explore_maze" in task_name.lower():
    success, metrics = self.explorer.explore(duration=600)
elif "quick_scan" in task_name.lower():
    success, metrics = self.explorer.explore(duration=60)
```

### Multi-Module Exploration

Launch multiple modules with different namespaces:

```bash
# Module 1
ros2 run coven_core module --ros-args -r __ns:=/robot1

# Module 2
ros2 run coven_core module --ros-args -r __ns:=/robot2
```

Dock will assign tasks to first available module.

### Visualize Exploration in RViz

```bash
ros2 run rviz2 rviz2
```

Add displays:
- Map (`/map`) - SLAM occupancy grid
- RobotModel - TurtleBot4 visualization
- LaserScan (`/scan`) - LiDAR data
- Path (`/plan`) - Nav2 global path
- Local Costmap - Obstacle avoidance

---

## Future Enhancements

Planned for Phase 2:

1. **Real hardware integration** - IR beacon docking for TurtleBot4
2. **Multi-resolution maps** - Coarse + fine detail
3. **Collaborative exploration** - Multiple modules coordinate
4. **Active SLAM** - Optimize viewpoints for mapping
5. **Semantic mapping** - Object recognition and labeling
6. **Energy-aware planning** - Battery monitoring and charging
7. **Map merging** - Combine maps from multiple modules

---

## Dependencies

**Python packages:**
```
numpy
scipy
```

**ROS2 packages:**
```
nav2_simple_commander
nav2_msgs
nav2_map_server
slam_toolbox
geometry_msgs
nav_msgs
tf2_ros
tf2_geometry_msgs
```

Install missing dependencies:
```bash
sudo apt install ros-jazzy-nav2-simple-commander \
                 ros-jazzy-nav2-map-server \
                 ros-jazzy-slam-toolbox \
                 python3-scipy python3-numpy
```

---

## References

- COVEN Core README: [README.md](README.md)
- Developer Guide: [README_DEV.md](README_DEV.md)
- Nav2 Documentation: https://navigation.ros.org/
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- TurtleBot4 Manual: https://turtlebot.github.io/turtlebot4-user-manual/

---

**Author:** Alexander Shultis
**Institution:** University of Hawaii at Manoa / Colorado School of Mines
**Date:** October 2025
**License:** MIT
