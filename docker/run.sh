#!/bin/bash
# COVEN-CORE Docker Runner Script
# Usage: ./run.sh [command]
# Examples:
#   ./run.sh              - Start interactive shell
#   ./run.sh build        - Build the workspace
#   ./run.sh sim          - Launch simulation
#   ./run.sh test         - Run tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Allow X11 connections from Docker
xhost +local:docker 2>/dev/null || true

# Export display variables for docker-compose
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

# Check if container image exists, build if not
if ! docker images | grep -q "coven-ros2-jazzy"; then
    echo -e "${YELLOW}Building COVEN Docker image (this may take 10-15 minutes)...${NC}"
    docker compose build
fi

case "${1:-shell}" in
    build)
        echo -e "${GREEN}Building COVEN workspace...${NC}"
        docker compose run --rm coven bash -c "
            source /opt/ros/jazzy/setup.bash && \
            cd /ros2_ws && \
            colcon build --symlink-install --packages-select coven_core && \
            echo -e '${GREEN}Build complete!${NC}'
        "
        ;;

    sim)
        echo -e "${GREEN}Launching COVEN simulation...${NC}"
        docker compose run --rm coven bash -c "
            source /opt/ros/jazzy/setup.bash && \
            source /ros2_ws/install/setup.bash 2>/dev/null || true && \
            cd /ros2_ws && \
            ros2 launch coven_core coven_sim.launch.py
        "
        ;;

    sim-headless)
        echo -e "${GREEN}Launching COVEN simulation (headless)...${NC}"
        docker compose run --rm coven bash -c "
            source /opt/ros/jazzy/setup.bash && \
            source /ros2_ws/install/setup.bash 2>/dev/null || true && \
            cd /ros2_ws && \
            ros2 launch coven_core coven_sim.launch.py headless:=true
        "
        ;;

    test)
        echo -e "${GREEN}Running COVEN tests...${NC}"
        docker compose run --rm coven bash -c "
            source /opt/ros/jazzy/setup.bash && \
            source /ros2_ws/install/setup.bash 2>/dev/null || true && \
            cd /ros2_ws && \
            colcon test --packages-select coven_core && \
            colcon test-result --verbose
        "
        ;;

    shell|*)
        echo -e "${GREEN}Starting COVEN development shell...${NC}"
        echo -e "${YELLOW}Tips:${NC}"
        echo "  - Build:  colcon build --symlink-install --packages-select coven_core"
        echo "  - Launch: ros2 launch coven_core coven_sim.launch.py"
        echo "  - Test:   colcon test --packages-select coven_core"
        echo ""
        docker compose run --rm coven
        ;;
esac
