# COVEN Exploration - Troubleshooting Guide

## Known Issues

### 1. Nav2 Planner Server Fails to Configure

**Symptom:**
```
[planner_server-49] [FATAL] Failed to create global planner. Exception:
According to the loaded plugin descriptions the class nav2_navfn_planner::NavfnPlanner
with base class type nav2_core::GlobalPlanner does not exist.
```

**Cause:**
- TurtleBot4 navigation config uses old-style plugin names (`::`) instead of new style (`/`)
- This is a known issue with TurtleBot4 Jazzy packages

**Workaround:**

Don't use the full `exploration_demo.launch.py` until TurtleBot4 packages are updated. Instead, launch components separately:

**Terminal 1 - Simulation:**
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

**Terminal 2 - Dock:**
```bash
ros2 run coven_core dock_multi
```

**Terminal 3 - Module (basic, no nav):**
```bash
ros2 run coven_core module
```

**Terminal 4 - Test mission:**
```bash
# This will use the fallback 5s delay (no actual navigation)
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test_mission\"}"}'
```

**What You Should See:**
1. Dock broadcasts IDENTIFY_REQ every 5 seconds
2. Module responds with IDENTIFY_REP
3. Dock sends VERIFY_REQ
4. Module replies VERIFY_REP
5. Dock enables power
6. Module enters NORMAL state and starts heartbeat
7. Module shows green heartbeat logs
8. When you send mission, module stops heartbeat, waits 5s, returns

### Fix for Nav2 Issue (if needed for exploration):

Edit `/home/ander/ros2_ws/src/turtlebot4/turtlebot4_navigation/config/nav2.yaml` line 224:

**Change from:**
```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
```

**Change to:**
```yaml
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot4_navigation
source install/setup.bash
```

---

## Testing Strategy

### Phase 1: Basic COVEN Functionality (No Navigation)

Test the core FSM and communication:

```bash
# Use the basic test script
~/ros2_ws/test_basic.sh
```

**Expected Result:**
- Module connects to dock
- Heartbeat shows green
- Mission triggers 5s delay task
- Module returns and resumes heartbeat

### Phase 2: SLAM Testing

Test map building without full navigation:

**Terminal 1:**
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

**Terminal 2:**
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

**Terminal 3 - Check map:**
```bash
ros2 topic echo /map --once
```

**Expected:** Map topic publishes occupancy grid

### Phase 3: Manual Navigation Testing

Test Nav2 separately:

**Fix nav2.yaml first (see above), then:**

```bash
# Terminal 1
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# Terminal 2
ros2 launch turtlebot4_navigation slam.launch.py

# Terminal 3
ros2 launch turtlebot4_navigation nav2.launch.py

# Terminal 4 - Send a goal manually
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

**Expected:** Robot navigates to goal

### Phase 4: Full Exploration (After Nav2 Works)

Once Nav2 is confirmed working, launch the full stack:

```bash
ros2 launch coven_core exploration_demo.launch.py
```

---

## Alternative: Simplified Exploration Without TurtleBot4 Nav2

If you want to test exploration logic without fixing TurtleBot4 configs, modify the exploration to use basic waypoint navigation instead of Nav2:

1. Create a simple "drive forward" behavior
2. Test frontier detection separately
3. Validate map saving/serialization

This would bypass Nav2 entirely for initial testing.

---

## Debugging Tools

### Check Running Nodes
```bash
ros2 node list
```

**Expected nodes:**
- `/coven_dock`
- `/coven_module`
- Plus all TurtleBot4 nodes

### Monitor COVEN Topics
```bash
# All COVEN communication
ros2 topic list | grep coven

# Watch heartbeat
ros2 topic echo /coven/heartbeat

# Watch task lifecycle
ros2 topic echo /coven/task_start
ros2 topic echo /coven/task_complete
```

### Check Module State
```bash
ros2 node info /coven_module
```

### Check if SLAM is Running
```bash
ros2 topic hz /map
```

**Expected:** ~5 Hz update rate

### Check if Nav2 is Ready
```bash
ros2 topic list | grep navigation
```

---

## Common Errors

### "Failed to initialize navigation"

**Cause:** Nav2 not ready when module tries to init

**Fix:** Wait longer before sending exploration mission (15-20 seconds)

### "No map available"

**Cause:** SLAM not publishing yet

**Fix:** Wait 30 seconds for SLAM to build initial map

### "scipy not available"

**Cause:** Missing scipy package

**Fix:**
```bash
pip install scipy
```

### "Map saver failed"

**Cause:** nav2_map_server not installed or not in PATH

**Fix:**
```bash
sudo apt install ros-jazzy-nav2-map-server
```

### Module doesn't move during exploration

**Causes:**
1. Nav2 not initialized
2. No frontiers detected (map too small)
3. Navigation goals unreachable

**Debug:**
```bash
# Check if Nav2 alive
ros2 node list | grep navigation

# Check map size
ros2 topic echo /map --once | grep -A 3 info

# Check for navigation errors
ros2 topic echo /diagnostics
```

---

## Success Criteria

### ✅ Phase 1 - Basic COVEN
- [ ] Dock broadcasts IDENTIFY_REQ
- [ ] Module connects (IDENTIFY → VERIFY → NORMAL)
- [ ] Heartbeat shows green every ~1s
- [ ] Mission triggers task execution
- [ ] Module completes and returns to NORMAL

### ✅ Phase 2 - SLAM
- [ ] /map topic publishes
- [ ] Occupancy grid contains data
- [ ] Map updates as robot moves (manual teleop)

### ✅ Phase 3 - Nav2
- [ ] All Nav2 nodes start without errors
- [ ] Manual goal pose triggers navigation
- [ ] Robot reaches goal successfully

### ✅ Phase 4 - Full Exploration
- [ ] Module initializes Nav2+SLAM
- [ ] Frontiers detected
- [ ] Robot navigates to frontiers
- [ ] Returns to dock
- [ ] Map saved and transmitted
- [ ] Dock stores map files

---

## Next Steps

1. **First:** Test basic COVEN (no nav) - verify FSM works
2. **Second:** Test SLAM alone - verify map building
3. **Third:** Fix Nav2 config - get navigation working
4. **Fourth:** Test full exploration mission

Don't try to test everything at once - validate each layer independently!

---

## Contact

If issues persist, check:
- ROS2 Jazzy version: `ros2 doctor`
- TurtleBot4 packages version: `ros2 pkg list | grep turtlebot4`
- Nav2 version: `ros2 pkg list | grep nav2`

Expected versions:
- ROS2: Jazzy Jalisco
- TurtleBot4: ~2.0+
- Nav2: ~1.1.x

---

**Author:** Alexander Shultis
**Date:** October 2025
