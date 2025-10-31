# COVEN Core Changelog

## 2025-10-30 - Simulation Mode Fixes

### Fixed: Simulation Mode Not Working
**Problem**: Modules connected in basic mode but immediately lost heartbeats in simulation mode.

**Root Cause**: Heartbeat tracking used Python's `time.time()` (wallclock) instead of ROS time. When `use_sim_time:=true`, ROS nodes use simulation time from Gazebo's `/clock` topic, creating a clock mismatch.

**Solution**: Changed all `time.time()` calls to `self.get_clock().now()` in dock_node_multi.py.

**Files Changed**:
- `coven_core/dock_node_multi.py` - Lines 107, 145, 152, 228

---

### Improved: Startup Timing
**Problem**: Blind `sleep` timers guessing how long components take to initialize.

**Solution**: Replaced all sleep timers with actual readiness checks:
- Wait for `/clock` topic publishing
- Wait for robot spawn (`/scan` topic)
- Wait for controller_manager services
- Wait for SLAM/Nav2 services
- Wait for COVEN topics

**Result**: Faster launches on fast systems, more reliable on slow systems.

**Files Changed**:
- `coven` launcher script
- `coven.launch.sh` launcher script

---

### Fixed: Messy Shutdown with Double Ctrl+C
**Problem**: Pressing Ctrl+C showed exception spam and required pressing Ctrl+C twice.

**Root Cause**: Race condition where `rclpy.shutdown()` was called twice - once by signal handler, once by our `finally` block.

**Solution**:
1. Wrapped `rclpy.spin()` in try/except to catch `KeyboardInterrupt`
2. Added `if rclpy.ok():` check before calling `rclpy.shutdown()`

**Result**: Clean one-tap Ctrl+C shutdown with no spam.

**Files Changed**:
- `coven_core/dock_node_multi.py` - Lines 317-326
- `coven_core/module_node.py` - Lines 330-339
- `coven` launcher - All cleanup functions
- `coven.launch.sh` launcher - All cleanup functions

---

## Summary

**What works now**:
- ✅ Basic mode (`-3`) - Always worked, still perfect
- ✅ Sim mode (`-3s`) - Now works perfectly with proper time sync
- ✅ Nav mode (`-3n`) - Ready for navigation testing
- ✅ Full mode (`-3f`) - Complete stack with fast startup
- ✅ Multi-module (`-5`, `-10`, etc.) - All modes support any number
- ✅ Flag combinations (`-3sn`, `-5f`, etc.) - Mix and match modes
- ✅ Clean shutdown - One Ctrl+C, no spam

**Testing**:
```bash
# Basic mode (no simulation)
coven -3

# Simulation mode
coven -3s

# Full stack
coven -5f
```

All modes now have:
- Smart readiness waiting
- Proper ROS time handling
- Graceful shutdown
- Clean error messages

---

## 2025-10-30 - Clean Logging

### Added: `-v` Verbose Flag
**Problem**: Gazebo, SLAM, and Nav2 spam hundreds of lines of startup logs.

**Solution**: Added `-v` verbose flag. By default, infrastructure logs are suppressed - only COVEN logs shown.

**Usage**:
```bash
# Clean output (default)
coven -3s

# Verbose output (all logs)
coven -v -3s
```
