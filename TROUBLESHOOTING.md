# COVEN Troubleshooting Guide

Common issues and solutions.

## Quick Fixes

### "Command not found: assemble"
```bash
# Option 1: Reload shell
source ~/.bashrc

# Option 2: Use local version
cd ~/ros2_ws
./coven test

# Option 3: Reinstall
cd ~/ros2_ws
chmod +x coven
mkdir -p ~/.local/bin
cp coven ~/.local/bin/assemble
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

### "pytest: command not found"
```bash
# Install pytest
sudo apt install python3-pytest

# Or use python3 -m
python3 -m pytest src/coven_core/test/ -v
```

### Tests Fail: Module Import Errors
```bash
# Rebuild workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Or use assemble
assemble rebuild
```

### Tests Fail: ROS2 Not Sourced
```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Or use assemble (does this automatically)
assemble test
```

## Test Issues

### Health Check Failures (Expected!)
If you see errors like:
```
Lidar offline - /scan topic not found
Nav2 initialization failed
```

This is **expected behavior** when running without hardware! COVEN correctly detects missing sensors and refuses to operate. The tests use `skip_health_check=true` to test coordination logic independently.

**Solution:** Use the test suite or `assemble` commands:
```bash
assemble test         # Tests use skip_health_check automatically
assemble module       # Also uses skip_health_check
```

### Tests Are Slow
Expected test times:
- Unit tests: ~0.6s (38 tests)
- Rigorous integration: ~26s (10 tests)
- Protocol validation: ~9s (6 tests)
- **Total: ~36s (54 tests)**

If much slower, check:
```bash
# Are other ROS2 nodes running?
ros2 node list

# Kill old nodes if needed
killall -9 dock module

# Clean old logs
assemble clean-logs
# OR
rm /tmp/coven*.log
```

### Specific Test Failures
```bash
# Run with verbose output
python3 -m pytest src/coven_core/test/ -vv

# Run specific test
python3 -m pytest src/coven_core/test/test_rigorous_integration.py::TestRigorousIdentification::test_identification_latency -vv

# Show print statements
python3 -m pytest src/coven_core/test/ -v -s
```

## Runtime Issues

### Dock Not Finding Modules
**Symptoms:**
- Dock sends IDENTIFY_REQ but no modules respond
- No IDENTIFY_REP messages

**Solutions:**
```bash
# 1. Check modules are running
ros2 node list | grep coven

# 2. Check topics
ros2 topic list | grep coven

# 3. Check if messages are being sent
ros2 topic echo /coven/identify_req

# 4. Restart dock and modules
killall -9 dock module
assemble dock          # Terminal 1
assemble module        # Terminal 2
```

### No Heartbeats
**Symptoms:**
- Module is running but no heartbeats on `/coven/heartbeat`

**Causes:**
1. Module stuck in WAIT_VERIFY (waiting for verification)
2. Module in FIELD_OPS (heartbeats paused during missions)
3. Module hasn't received power enable

**Solutions:**
```bash
# Check module state in logs
assemble logs

# Look for:
# [INFO] [module]: Heartbeat started for <ID>

# If stuck in WAIT_VERIFY, check dock is sending VERIFY_REQ
ros2 topic echo /coven/verify_req
```

### Heartbeat Frequency Wrong
**Expected:** 1.25 Hz (0.8s period)

**Check actual rate:**
```bash
# Terminal 1
assemble module

# Terminal 2
ros2 topic hz /coven/heartbeat

# Should show ~1.25 Hz
```

**If wrong:** Check `hb_period` parameter in module_node.py (line 77-78)

### Multiple Modules with Same ID
**Symptom:**
```
[ERROR] [dock]: Multiple modules claiming ID RR-001
```

**Solution:** Use unique IDs:
```bash
assemble module RR-001     # Terminal 1
assemble module RR-002     # Terminal 2
assemble module RR-003     # Terminal 3
```

Or use auto-ID:
```bash
assemble module            # Gets auto-generated ID
```

## Build Issues

### colcon build Fails
```bash
# Check dependencies
sudo apt update
sudo apt install ros-humble-desktop python3-pytest

# Clean and rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install

# Or use assemble
assemble rebuild
```

### Python Import Errors After Build
```bash
# Source the workspace
cd ~/ros2_ws
source install/setup.bash

# Or use assemble (auto-sources)
assemble test
```

### "Package not found" Errors
```bash
# Check package built successfully
cd ~/ros2_ws
colcon list | grep coven_core

# Should show:
# coven_core   /home/ander/ros2_ws/src/coven_core

# If missing, rebuild
assemble rebuild
```

## Performance Issues

### High CPU Usage
**Normal:** Dock + multiple modules should use <10% CPU total

**If high:**
```bash
# Check what's running
top
# OR
htop

# Look for runaway processes
# Kill if needed
killall -9 dock module
```

### High Memory Usage
**Normal:** Dock + module ~100MB each

**If high:** Restart nodes

### Slow Message Delivery
**Expected:** Identification latency <100ms (actual: ~4ms)

**If slow:**
```bash
# Check system load
uptime

# Check network (if using DDS over network)
ros2 doctor

# Check topic statistics
ros2 topic bw /coven/heartbeat
ros2 topic hz /coven/heartbeat
```

## ROS2 Issues

### DDS Discovery Problems
```bash
# Check ROS2 daemon
ros2 daemon status

# Restart if needed
ros2 daemon stop
ros2 daemon start
```

### Topic Not Visible
```bash
# List all topics
ros2 topic list

# Should see:
# /coven/heartbeat
# /coven/identify_req
# /coven/identify_rep
# /coven/verify_req
# /coven/verify_rep
# /coven/task_req
# /coven/task_ack
# /coven/task_complete

# If missing, check nodes are running
ros2 node list
```

### QoS Mismatch
COVEN uses:
- **RELIABLE** for all command/control topics
- **Default QoS depth: 10**

If you're subscribing externally and not seeing messages:
```bash
# Use reliable QoS
ros2 topic echo /coven/heartbeat --qos-reliability reliable
```

## Documentation & Help

### Where to Find Help

| Issue Type | Resource |
|------------|----------|
| Quick commands | [TESTING_QUICK_REFERENCE.md](../../TESTING_QUICK_REFERENCE.md) |
| Test details | [TEST_ME.md](../../TEST_ME.md) |
| Test philosophy | [RIGOROUS_TESTING.md](../../RIGOROUS_TESTING.md) |
| All commands | [ASSEMBLE_GUIDE.md](../../ASSEMBLE_GUIDE.md) |
| Getting started | [QUICK_START.md](QUICK_START.md) |
| Main docs | [README.md](../../README.md) |

### Logs

Check logs when debugging:
```bash
# View recent logs
assemble logs

# Tail specific log
tail -f /tmp/coven_dock.log
tail -f /tmp/coven_module_1.log

# Clean old logs
assemble clean-logs
```

### Enable Debug Output
Edit module_node.py or dock_node.py, change logging level:
```python
# In __init__
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

## Known Issues

### Nav2 Integration
**Status:** ⏸ Blocked by Nav2 issues (independent of COVEN)

COVEN coordination is production-ready and fully tested. Navigation integration pending Nav2 fixes.

**Workaround:** Test coordination independently using `skip_health_check=true` (already done in test suite and `assemble` commands).

### Simulation
**Status:** Not currently used for testing

Tests run without Gazebo/simulation. When Nav2 is fixed, simulation-based tests can be added as another validation layer.

## Still Stuck?

1. Check you've run: `assemble test` (proves system works)
2. Review recent logs: `assemble logs`
3. Try clean rebuild: `assemble rebuild`
4. Check ROS2 basics: `ros2 doctor`

---

**Most common fix:** `assemble rebuild && assemble test`
