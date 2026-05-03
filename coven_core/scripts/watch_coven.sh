#!/bin/bash
# Watch COVEN dock topics in real-time
# Rovers communicate via UART only when physically docked.
# These ROS2 topics are internal to the dock.

echo "Watching COVEN dock topics..."
echo "Press Ctrl+C to stop"
echo ""

MODE=${1:-all}

case "$MODE" in
    bridge)
        echo "=== Rover Bridge Events ==="
        ros2 topic echo /coven/bridge_events &
        ros2 topic echo /coven/sensor_batch &
        wait
        ;;
    slam)
        echo "=== SLAM Processor ==="
        ros2 topic echo /coven/slam_processor_status &
        ros2 topic echo /map --field info &
        wait
        ;;
    dispatch)
        echo "=== Frontier Dispatcher ==="
        ros2 topic echo /coven/dispatcher_status &
        ros2 topic echo /coven/mission_request &
        wait
        ;;
    sim)
        echo "=== COVEN Simulation Bridge ==="
        echo ""
        echo "Sim-specific monitoring: dock-side handshake events plus the"
        echo "Gazebo-bridged rover topics the sim_rover_proxy is reading."
        echo ""
        echo "Press Ctrl+C to stop."
        ros2 topic echo /coven/bridge_events &
        ros2 topic echo /coven/rover_status &
        # If you launched the sim with rover_name=witch_morgan, these will exist:
        ros2 topic hz /witch_morgan/scan 2>/dev/null &
        ros2 topic hz /witch_morgan/odometry 2>/dev/null &
        wait
        ;;
    all|*)
        echo "=== COVEN Dock Topics ==="
        echo ""
        echo "Bridge:"
        echo "  /coven/bridge_events     - Rover connect/disconnect/verify"
        echo "  /coven/sensor_batch      - Data batch arrivals"
        echo "  /coven/rover_status      - Rover heartbeats"
        echo ""
        echo "SLAM:"
        echo "  /coven/slam_processor_status - SLAM processing state"
        echo "  /map                         - Occupancy grid"
        echo ""
        echo "Dispatch:"
        echo "  /coven/dispatcher_status   - Exploration progress"
        echo "  /coven/mission_request     - Mission assignments"
        echo "  /coven/auctioneer_status   - Auction state"
        echo ""
        echo "Modes: bridge | slam | dispatch | sim | all"
        echo ""
        echo "Monitoring bridge events (press Ctrl+C to stop)..."
        ros2 topic echo /coven/bridge_events
        ;;
esac
