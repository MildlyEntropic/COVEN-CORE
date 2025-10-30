#!/bin/bash
# Watch COVEN logs in real-time, filtering out noise

echo "Watching COVEN logs..."
echo "Press Ctrl+C to stop"
echo ""

# If no argument, watch all COVEN topics
MODE=${1:-all}

case "$MODE" in
    dock)
        ros2 topic echo /coven/identify_req &
        ros2 topic echo /coven/verify_req &
        ros2 topic echo /coven/enable_12v &
        wait
        ;;
    module)
        ros2 topic echo /coven/identify_rep &
        ros2 topic echo /coven/verify_rep &
        ros2 topic echo /coven/heartbeat &
        wait
        ;;
    all|*)
        echo "=== COVEN Communication ==="
        echo ""
        echo "Connection Topics:"
        echo "  /coven/identify_req  - Dock broadcasts"
        echo "  /coven/identify_rep  - Module responds"
        echo "  /coven/verify_req    - Dock verifies"
        echo "  /coven/verify_rep    - Module confirms"
        echo "  /coven/enable_12v    - Dock enables power"
        echo ""
        echo "Runtime Topics:"
        echo "  /coven/heartbeat     - Module health"
        echo ""
        echo "Task Topics:"
        echo "  /coven/task_req      - Task assignment"
        echo "  /coven/task_ack      - Task accepted"
        echo "  /coven/task_start    - Task begins"
        echo "  /coven/task_complete - Task done"
        echo ""
        echo "Monitoring heartbeat (press Ctrl+C to stop)..."
        ros2 topic echo /coven/heartbeat
        ;;
esac
