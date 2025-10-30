# COVEN Quick Start

## Installation

```bash
cd ~/ros2_ws
./assemble.sh  # Clean build + source
```

## Launch Options

### Basic Testing (1 Module)
```bash
./coven
```
**Use for**: Testing core FSM, development, clear logs

### Multi-Module Testing (N Modules)
```bash
./coven -3   # 3 modules
./coven -5   # 5 modules
./coven -10  # 10 modules (stress test)
```
**Use for**: Swarm coordination, load testing, multi-agent demos

### With Simulation
```bash
./coven sim
```
**Use for**: Testing with Create3 robot in Gazebo

### With Navigation
```bash
./coven nav
```
**Use for**: SLAM + Nav2 testing (requires robot/sim already running)

### Full Stack
```bash
./coven full
```
**Use for**: Complete exploration missions (sim + nav + COVEN)

## Testing Workflow

### 1. Test Basic Connection (1 Module)
```bash
# Terminal 1
./coven

# Terminal 2 - Watch heartbeat
ros2 topic echo /coven/heartbeat

# Terminal 3 - Send test mission
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"test\"}"}'
```

### 2. Test Multi-Module Coordination (3 Modules)
```bash
# Terminal 1
./coven -3

# Terminal 2 - Watch all heartbeats
ros2 topic echo /coven/heartbeat

# Terminal 3 - Send multiple missions
for i in {1..3}; do
  ros2 topic pub --once /coven/mission_req std_msgs/String \
    "{data: \"{\\\"task\\\": \\\"mission_$i\\\"}\"}"
  sleep 1
done
```

### 3. Test Exploration Mission (Full Stack)
```bash
# Terminal 1
./coven full

# Wait for everything to initialize (~30 seconds)

# Terminal 2 - Send exploration mission
ros2 topic pub --once /coven/mission_req std_msgs/String \
  '{data: "{\"task\": \"explore_warehouse\"}"}'

# Terminal 3 - Monitor completion
ros2 topic echo /coven/task_complete
```

## Monitoring

### Watch All COVEN Topics
```bash
ros2 topic list | grep coven
```

### Watch Connection Process
```bash
ros2 topic echo /coven/identify_req   # Dock broadcasts
ros2 topic echo /coven/identify_rep   # Module responds
ros2 topic echo /coven/verify_req     # Dock verifies
ros2 topic echo /coven/verify_rep     # Module confirms
```

### Watch Task Execution
```bash
ros2 topic echo /coven/task_start     # Task begins
ros2 topic echo /coven/task_complete  # Task done + map data
```

### Check Node Status
```bash
ros2 node list                # All running nodes
ros2 node info /coven_dock    # Dock details
ros2 topic hz /coven/heartbeat  # Heartbeat rate
```

## Common Scenarios

### Development
```bash
./coven           # Start simple
# Edit code
./assemble.sh     # Rebuild
./coven -2        # Test with 2 modules
```

### Demo
```bash
./coven -5        # Launch 5 modules
# Show swarm connecting, heartbeating, task distribution
```

### Research
```bash
./coven -10       # Launch 10 modules
# Study emergent behaviors, task allocation
```

### Integration Testing
```bash
./coven full      # Full stack
# Test navigation, SLAM, map transfer end-to-end
```

## Stopping

Press **Ctrl+C** in the terminal running `./coven` - it will cleanly stop all nodes (dock + all modules).

## Troubleshooting

### "Package not found"
```bash
cd ~/ros2_ws
./assemble.sh
```

### "No heartbeat received"
- Check `ros2 node list` - is dock running?
- Check `ros2 topic list | grep coven` - are topics present?
- Increase IDENTIFY broadcast rate in [dock_node_multi.py](coven_core/dock_node_multi.py#L50)

### "Navigation failed"
- Ensure `./coven full` (not just `./coven`)
- Check Nav2 nodes: `ros2 node list | grep nav`
- View costmap: `ros2 run nav2_costmap_2d nav2_costmap_2d_markers`

### "Map data not saved"
- Check `~/coven_maps/` directory exists
- Module must complete exploration before dock saves map
- Check logs for "Map data from RR-xxxxx saved"

## File Locations

- **Launch script**: `/home/ander/ros2_ws/coven`
- **Package source**: `/home/ander/ros2_ws/src/coven_core/`
- **Map storage**: `~/coven_maps/`
- **Build output**: `/home/ander/ros2_ws/install/coven_core/`

## Documentation

- [MULTI_MODULE.md](MULTI_MODULE.md) - Multi-module testing guide
- [EXPLORATION.md](EXPLORATION.md) - Exploration system details
- [VIEWING_LOGS.md](VIEWING_LOGS.md) - Log monitoring tips
- [LAUNCHER.md](LAUNCHER.md) - Detailed launcher usage
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Common issues

## Next Steps

1. **Test basic FSM**: `./coven` - verify connection, heartbeat, task execution
2. **Test multi-module**: `./coven -3` - verify multiple modules connect and distribute tasks
3. **Test navigation**: `./coven full` - verify exploration missions work end-to-end
4. **Integrate your missions**: Modify [module_node.py](coven_core/module_node.py) to add custom task types

---

**TL;DR**: Use `./coven` for basic testing, `./coven -N` for multi-module testing, `./coven full` for complete exploration missions. Press Ctrl+C to stop. Check [MULTI_MODULE.md](MULTI_MODULE.md) for detailed examples.
