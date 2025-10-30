# Multi-Module Testing

## Quick Start

Launch COVEN with multiple modules:

```bash
# 1 module (default)
./coven

# 3 modules
./coven -3

# 5 modules
./coven -5

# 10 modules (stress test!)
./coven -10
```

---

## What You'll See

When you run `./coven -3`, you get:

```
================================================
  COVEN Basic Mode
  Launching: Dock + 3 Module(s)
  No simulation, no navigation
================================================

Started dock (PID: 12345)
Started module 1 (PID: 12346)
Started module 2 (PID: 12347)
Started module 3 (PID: 12348)

All nodes started!
  Dock PID: 12345
  Module PIDs: 12346 12347 12348

Press Ctrl+C to stop
```

Then the beautiful logs start flowing:

```
[INFO] [coven_dock]: DockMulti initialized — ready to manage multiple modules.
[INFO] [coven_dock]: Broadcast IDENTIFY_REQ dock_broadcast
[INFO] [coven_module]: Module RR-a3b4c5 booted on 5V — waiting for IDENTIFY.
[INFO] [coven_module]: Module RR-f7e821 booted on 5V — waiting for IDENTIFY.
[INFO] [coven_module]: Module RR-9d2c14 booted on 5V — waiting for IDENTIFY.
[INFO] [coven_dock]: IDENTIFY_REP received from RR-a3b4c5
[INFO] [coven_dock]: IDENTIFY_REP received from RR-f7e821
[INFO] [coven_dock]: IDENTIFY_REP received from RR-9d2c14
[INFO] [coven_dock]: Heartbeat received for RR-a3b4c5, RR-f7e821, RR-9d2c14
...
```

All three modules connecting, heartbeating, and ready for missions!

---

## Testing Multi-Module Missions

### Send a Mission

The dock will assign it to the first available module:

```bash
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

**You'll see:**
- Dock assigns task to one module (e.g., RR-a3b4c5)
- That module stops heartbeat and executes
- Other modules continue heartbeating normally
- When task completes, all three resume heartbeating

### Send Multiple Missions

Send missions back-to-back to test task distribution:

```bash
# Mission 1
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_area_1\"}"}'

# Wait 1 second, mission 2
sleep 1
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_area_2\"}"}'

# Wait 1 second, mission 3
sleep 1
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_area_3\"}"}'
```

**Result:**
- Module 1 gets task "explore_area_1"
- Module 2 gets task "explore_area_2"
- Module 3 gets task "explore_area_3"
- All three execute in parallel!

---

## Watch Heartbeats

In another terminal, monitor all heartbeats:

```bash
ros2 topic echo /coven/heartbeat
```

You'll see messages cycling through all module IDs:

```
data: '{"module_id": "RR-a3b4c5", "seq": 42}'
---
data: '{"module_id": "RR-f7e821", "seq": 38}'
---
data: '{"module_id": "RR-9d2c14", "seq": 51}'
---
data: '{"module_id": "RR-a3b4c5", "seq": 43}'
---
```

All modules reporting in!

---

## Stress Testing

### Test Connection Handling

Launch 10 modules to test the dock's multi-module management:

```bash
./coven -10
```

Watch the dock:
- Handle 10 IDENTIFY_REP messages
- Verify all 10 modules
- Enable power for all 10
- Monitor 10 heartbeat streams
- Log "Heartbeat received for RR-xxx, RR-yyy, RR-zzz, ..." (all 10!)

### Test Task Distribution

With 10 modules running, send 5 missions rapidly:

```bash
for i in {1..5}; do
  ros2 topic pub --once /coven/mission_req std_msgs/String \
    "{data: \"{\\\"task\\\": \\\"mission_$i\\\"}\"}" &
done
```

**Result:**
- Dock assigns 5 different modules
- 5 modules execute tasks in parallel
- 5 modules continue heartbeating (available)
- As tasks complete, modules return to pool

---

## Failure Testing

### Test Heartbeat Dropout

While running multiple modules, kill one:

```bash
# Find a module PID (shown at startup)
kill 12347

# Or use pkill
pkill -f "ros2 run coven_core module" | head -1
```

**Watch the logs:**
```
[WARN] Heartbeat missing ONCE for RR-f7e821     # Yellow
[WARN] Heartbeat missing TWICE for RR-f7e821    # Orange
[ERROR] Heartbeat lost from RR-f7e821           # Red
```

The other modules continue normally!

### Test Recovery

Kill and restart a module:

```bash
# Kill one
kill 12347

# Start new one (after 3+ seconds so it's dropped)
sleep 5
ros2 run coven_core module &
```

**Watch:**
- Old module detected as lost (red)
- New module connects with new ID
- Dock verifies and adds to pool
- New module starts heartbeating

---

## Performance Metrics

### Connection Time

With N modules, expect:
- **N × 5s** for all to connect (staggered IDENTIFY broadcasts)
- **~1s** per module for VERIFY handshake
- Total: **~5-10 seconds** for all modules online

### Heartbeat Load

Each module sends heartbeat every 0.8s:
- 1 module: **1.25 msg/s**
- 3 modules: **3.75 msg/s**
- 10 modules: **12.5 msg/s**

Dock aggregates and logs every 0.5s.

### Task Throughput

With N modules:
- **Parallel execution:** Up to N tasks simultaneously
- **Sequential execution:** One per module, round-robin

---

## Use Cases

### Research: Swarm Coordination
```bash
./coven -10
# Test emergent behaviors with 10 independent agents
```

### Development: Load Testing
```bash
./coven -20
# Stress test message handling and task distribution
```

### Demo: Multi-Agent System
```bash
./coven -5
# Show 5 modules connecting, heartbeating, executing tasks
```

### Testing: Edge Cases
```bash
./coven -3
# Kill modules randomly, test recovery
# Send conflicting missions, test arbitration
```

---

## Limitations

### Current Implementation

**Per-module constraints:**
- Each module gets unique ID (UUID-based)
- Each module runs in separate process
- Each module has independent FSM state

**Dock constraints:**
- Manages unlimited modules (tested to 50+)
- First-available task assignment
- No priority/capability-based selection (yet)

### System Constraints

**ROS2 / OS limits:**
- Process limits (~1000 on typical Linux)
- File descriptor limits (~1024 default)
- Network buffer sizes (affects latency at scale)

**Practical limits:**
- **1-10 modules:** Smooth, real-time logs
- **10-50 modules:** Manageable, logs scroll fast
- **50+ modules:** CPU/network constrained, logs chaotic

---

## Best Practices

### Development

Start small, scale up:
```bash
./coven     # Test with 1
./coven -3  # Verify multi-module logic
./coven -10 # Stress test
```

### Testing

Use specific numbers for specific tests:
```bash
./coven -2  # Test task distribution (minimum)
./coven -3  # Test heartbeat aggregation
./coven -5  # Test parallel mission execution
./coven -10 # Test connection scalability
```

### Debugging

Fewer modules = clearer logs:
```bash
./coven -2  # Easy to follow in logs
```

### Demos

Impressive but not overwhelming:
```bash
./coven -5  # Shows swarm capability, logs readable
```

---

## Advanced: Heterogeneous Modules

**Future enhancement:** Different module types

```bash
# Current (all ReconRover)
./coven -3

# Future (mixed types)
./coven --modules "ReconRover:2,SensorPod:1,ChargeStation:1"
```

Could specify:
- Module types (ReconRover, SensorPod, etc.)
- Capabilities (exploration, sensing, charging)
- Task affinity (assign exploration to ReconRovers only)

---

## Quick Reference

```bash
./coven             # 1 module
./coven -3          # 3 modules
./coven -10         # 10 modules (stress test)

# Monitor
ros2 topic echo /coven/heartbeat           # Watch all heartbeats
ros2 topic echo /coven/task_complete       # Watch task completions

# Test missions
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'

# Stop all
Ctrl+C              # Kills dock + all modules
```

---

**TL;DR:** Use `./coven -N` to launch dock + N modules. Watch them all connect, heartbeat, and execute missions in parallel. Perfect for testing swarm coordination!
