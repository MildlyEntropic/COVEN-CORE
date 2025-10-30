# Viewing COVEN Logs

## The Problem

When you run `./coven full`, you launch **40+ nodes** (simulation + navigation + COVEN). All their logs get mixed together, making it hard to see COVEN-specific activity.

---

## Solution 1: Run COVEN Basic Mode

The **basic mode has ONLY 2 nodes** (dock + module), so logs are crystal clear:

```bash
./coven
```

**You'll see:**
```
[INFO] [coven_dock]: DockMulti initialized — ready to manage multiple modules.
[INFO] [coven_dock]: Broadcast IDENTIFY_REQ dock_broadcast
[INFO] [coven_module]: Module RR-a3b4c5 booted on 5V — waiting for IDENTIFY.
[INFO] [coven_module]: IDENTIFY_REQ received → responding with module ID
[INFO] [coven_dock]: IDENTIFY_REP received from RR-a3b4c5
[INFO] [coven_dock]: VERIFY_REP OK for RR-a3b4c5 → enabling +12V
[INFO] [coven_module]: +12V enabled → entering NORMAL state
[INFO] [coven_module]: Heartbeat started for RR-a3b4c5
[INFO] [coven_dock]: Heartbeat received for RR-a3b4c5
```

**Clear and easy to follow!**

---

## Solution 2: Watch Specific Topics

Instead of watching ALL logs, monitor COVEN topics directly:

### Watch Heartbeat
```bash
ros2 topic echo /coven/heartbeat
```

Output:
```
data: '{"module_id": "RR-a3b4c5", "seq": 42}'
---
data: '{"module_id": "RR-a3b4c5", "seq": 43}'
---
```

### Watch Connection Process
```bash
# Terminal 1
ros2 topic echo /coven/identify_req

# Terminal 2
ros2 topic echo /coven/identify_rep

# Terminal 3
ros2 topic echo /coven/verify_req

# Terminal 4
ros2 topic echo /coven/verify_rep
```

### Watch Task Execution
```bash
# Terminal 1
ros2 topic echo /coven/task_start

# Terminal 2
ros2 topic echo /coven/task_complete
```

---

## Solution 3: Filter Launch Output

When running full mode, pipe through grep to see only COVEN:

```bash
./coven full 2>&1 | grep -E "(coven_dock|coven_module|COVEN)"
```

Or save all logs to a file and search later:
```bash
./coven full 2>&1 | tee coven_full.log

# Later, search the log
grep "coven_dock" coven_full.log
grep "Heartbeat" coven_full.log
grep "IDENTIFY" coven_full.log
```

---

## Solution 4: Use ROS2 CLI Tools

### List All COVEN Topics
```bash
ros2 topic list | grep coven
```

Output:
```
/coven/enable_12v
/coven/heartbeat
/coven/identify_rep
/coven/identify_req
/coven/mission_req
/coven/task_ack
/coven/task_complete
/coven/task_req
/coven/task_start
/coven/verify_rep
/coven/verify_req
```

### Check Topic Rates
```bash
ros2 topic hz /coven/heartbeat
```

Output:
```
average rate: 1.250
    min: 0.800s max: 0.801s std dev: 0.00050s window: 10
```

### Check Node Info
```bash
ros2 node info /coven_dock
```

Shows all topics, services, and parameters for the dock.

---

## Solution 5: Use RQT

Visual monitoring tool:

```bash
rqt
```

1. Go to **Plugins → Topics → Topic Monitor**
2. Check boxes next to `/coven/*` topics
3. See real-time message rates and data

Or use the message viewer:
1. **Plugins → Topics → Message Publisher**
2. Select `/coven/mission_req`
3. Manually send missions with GUI

---

## What Logs Look Like

### Connection Sequence

**Dock broadcasts every 5 seconds:**
```
[INFO] [coven_dock]: Broadcast IDENTIFY_REQ dock_broadcast
```

**Module responds:**
```
[INFO] [coven_module]: IDENTIFY_REQ received → responding with module ID
[INFO] [coven_dock]: IDENTIFY_REP received from RR-a3b4c5
```

**Dock verifies:**
```
[INFO] [coven_dock]: VERIFY_REQ matched → replying OK
[INFO] [coven_module]: +12V enabled → entering NORMAL state
```

**Heartbeat starts:**
```
[INFO] [coven_module]: Heartbeat started for RR-a3b4c5
[INFO] [coven_dock]: Heartbeat received for RR-a3b4c5
```

### Task Execution

**Mission sent:**
```
[INFO] [coven_dock]: Assigned mission 'explore_warehouse' to RR-a3b4c5
[INFO] [coven_module]: TASK_REQ received: explore_warehouse
[INFO] [coven_module]: Accepted task → preparing to undock
```

**Execution:**
```
[INFO] [coven_module]: Executing FIELD_OPS for task: explore_warehouse
[INFO] [coven_module]: Heartbeat stopped for RR-a3b4c5
[INFO] [coven_dock]: RR-a3b4c5 started FIELD_OPS: explore_warehouse
```

**Completion:**
```
[INFO] [coven_module]: Task explore_warehouse complete — rejoined dock
[INFO] [coven_dock]: TaskComplete from RR-a3b4c5: explore_warehouse → SUCCESS
[INFO] [coven_dock]: Map data from RR-a3b4c5 saved to ~/coven_maps/...
```

---

## Recommended Workflow

### For Development (Clear Logs)
```bash
# Terminal 1 - Run basic COVEN
./coven

# Terminal 2 - Watch heartbeat
ros2 topic echo /coven/heartbeat

# Terminal 3 - Send test missions
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

### For Testing Navigation (Filtered Logs)
```bash
# Terminal 1 - Run full stack with log filter
./coven full 2>&1 | grep -E "(coven_|Heartbeat|IDENTIFY|TASK)"

# Terminal 2 - Monitor specific topics
ros2 topic echo /coven/task_complete
```

### For Demos (Full Output)
```bash
# Let it all show - impressive!
./coven full
```

---

## Why Logs Got "Lost"

When you ran `./coven full` earlier:
- **40+ nodes launched** (Gazebo, Nav2, SLAM, COVEN)
- All printing to same terminal
- COVEN logs **were there**, just buried
- Nav2 is VERY chatty (lifecycle transitions, costmaps, etc.)

**The logs weren't lost - they were just mixed in!**

---

## Quick Reference

### See COVEN Communication
```bash
./coven                     # Basic mode, clear logs
ros2 topic list | grep coven  # List COVEN topics
ros2 topic echo /coven/heartbeat  # Watch specific topic
```

### Debug Connection Issues
```bash
ros2 node list              # Check nodes running
ros2 node info /coven_dock  # Inspect dock node
ros2 topic hz /coven/identify_req  # Check broadcast rate
```

### Monitor Mission Execution
```bash
ros2 topic echo /coven/task_start    # See when task begins
ros2 topic echo /coven/task_complete # See completion + data
```

---

**TL;DR:** The logs are there! Use `./coven` (basic mode) for clear output, or use `ros2 topic echo` to watch specific COVEN topics when running full mode.
