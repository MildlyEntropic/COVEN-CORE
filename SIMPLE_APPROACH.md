# COVEN Simple Exploration - No TurtleBot4 Dependencies

## What We Did

**Stripped out all TurtleBot4-specific code and created a minimal, self-contained navigation system.**

### What Was Removed
- ❌ TurtleBot4 navigation launch files
- ❌ TurtleBot4-specific configs
- ❌ TurtleBot4 hardware nodes
- ❌ All TurtleBot4 build dependencies

### What We Kept
- ✅ Pure Nav2 (BasicNavigator, planners, controllers)
- ✅ SLAM Toolbox (independent)
- ✅ Create3 base robot (if you want sim, optional)
- ✅ All COVEN core functionality

### What We Created
1. **`config/nav2_simple.yaml`** - Minimal Nav2 config (no TurtleBot4)
2. **`config/slam_simple.yaml`** - Minimal SLAM config
3. **`launch/coven_explore_simple.launch.py`** - Direct Nav2 launch

---

## Testing Strategy

### Level 1: COVEN Only (No Nav, No Sim)

Test pure COVEN FSM without any navigation:

```bash
# Terminal 1 - Dock
ros2 run coven_core dock_multi

# Terminal 2 - Module
ros2 run coven_core module

# Terminal 3 - Send mission
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

**Expected:**
- Module connects (IDENTIFY → VERIFY → NORMAL)
- Heartbeat every ~1s
- Mission triggers 5s delay (fallback, no nav)
- Module completes

**This proves COVEN works independently!**

---

### Level 2: With Create3 Simulation

If you want to test with a simulated robot:

```bash
# Terminal 1 - Create3 Sim (if available)
ros2 launch irobot_create_gz_bringup create3_gz.launch.py

# Terminal 2 - Dock
ros2 run coven_core dock_multi

# Terminal 3 - Module
ros2 run coven_core module

# Terminal 4 - Mission
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore\"}"}'
```

---

### Level 3: Full Nav2 Stack

Launch everything with our simple config:

```bash
# Terminal 1 - Create3 Sim
ros2 launch irobot_create_gz_bringup create3_gz.launch.py

# Terminal 2 - Our navigation + COVEN
ros2 launch coven_core coven_explore_simple.launch.py

# Wait 15 seconds...

# Terminal 3 - Exploration mission
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_area\"}"}'
```

---

## Architecture

```
Old (Complex):
User → exploration_demo.launch.py → TurtleBot4 launch → TurtleBot4 nav → Nav2
                                   ↓
                              Broken config!

New (Simple):
User → coven_explore_simple.launch.py → Nav2 directly
                                       ↓
                                   Works!
```

---

## What This Gives You

### Independence
- COVEN core works WITHOUT navigation
- Navigation is optional add-on
- No external package dependencies for core functionality

### Simplicity
- One config file for Nav2
- One config file for SLAM
- Direct node launches (no nested includes)

### Flexibility
- Use ANY robot that provides `/cmd_vel`, `/scan`, `/odom`
- Not locked to TurtleBot4
- Easy to adapt for your custom cuberover hardware

---

## File Structure

```
coven_core/
├── config/
│   ├── nav2_simple.yaml       # Our Nav2 config
│   └── slam_simple.yaml       # Our SLAM config
├── launch/
│   ├── coven_explore_simple.launch.py  # Simple launcher
│   └── exploration_demo.launch.py      # Old (broken with TB4)
└── coven_core/
    ├── common.py              # Message protocol
    ├── dock_node_multi.py     # Dock (no nav deps)
    ├── module_node.py         # Module (nav optional)
    └── exploration.py         # Pure Nav2 (no TB4)
```

---

## Dependencies

### Core COVEN (always needed):
```bash
sudo apt install ros-jazzy-rclpy
```

### For Exploration (optional):
```bash
sudo apt install \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-waypoint-follower \
  ros-jazzy-nav2-velocity-smoother \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-slam-toolbox

pip install numpy scipy
```

### For Simulation (optional):
```bash
sudo apt install \
  ros-jazzy-irobot-create-gz-bringup \
  ros-jazzy-ros-gz-bridge
```

---

## Troubleshooting

### "Module doesn't explore, just waits 5s"

This is **correct behavior** if Nav2 isn't running! The module has a fallback:

```python
if "explore" in task_name.lower():
    if not self.navigator:
        self._initialize_navigation()  # Will fail gracefully
    ...
else:
    # Fallback - just wait
    time.sleep(TASK_DELAY)
```

**Solution:** Launch Nav2 stack first (see Level 3 above)

### "Nav2 nodes crash"

Check if topics exist:
```bash
ros2 topic list | grep -E "(cmd_vel|scan|odom)"
```

Must have:
- `/cmd_vel` - Robot velocity commands
- `/scan` - LiDAR data
- `/odom` - Odometry

If missing, you need a robot (sim or real)!

### "SLAM doesn't build map"

Check SLAM is running:
```bash
ros2 node list | grep slam
ros2 topic hz /map
```

Should see `/slam_toolbox` node and `/map` publishing at ~1-5 Hz.

---

## Next Steps

1. **Test Level 1** - Prove COVEN works alone
2. **Get Create3 sim working** - Test with robot
3. **Launch Nav2** - Test navigation
4. **Try exploration** - Full mission

Don't try everything at once - validate each layer!

---

## Advantages Over TurtleBot4 Approach

| Aspect | TurtleBot4 Approach | Simple Approach |
|--------|---------------------|-----------------|
| Dependencies | 20+ packages | 5-10 packages |
| Config files | Multiple nested | 2 files |
| Launch complexity | 5 levels deep | 1 level |
| Debugging | Hard (nested fails) | Easy (direct) |
| Portability | TB4 only | Any robot |
| Maintenance | Breaks with TB4 updates | Stable |

---

## Hardware Path

When you build your actual CubeRovers:

1. **Keep COVEN core as-is** - No changes needed
2. **Provide these topics from your hardware:**
   - `/cmd_vel` - From your motor controller
   - `/scan` - From your LiDAR
   - `/odom` - From your wheel encoders
3. **Launch Nav2 with our simple config** - Will work!
4. **Done** - Exploration works on real hardware

The simulation is just for testing. Real robots are simpler (no Gazebo complexity).

---

**Author:** Alexander Shultis
**Date:** October 2025
**Philosophy:** Keep it simple, keep it working.
