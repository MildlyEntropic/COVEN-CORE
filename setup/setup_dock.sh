#!/bin/bash
# =============================================================================
# setup_dock.sh — Set up the Pi 4 as the COVEN dock
# =============================================================================
#
# Run this ON the Pi 4 after first boot:
#   ssh coven@bene-gesserit.local
#   curl -fsSL <your-repo-url>/setup/setup_dock.sh | bash
#   — or —
#   scp setup_dock.sh coven@bene-gesserit.local:~ && ssh coven@bene-gesserit.local ./setup_dock.sh
#
# What this does:
#   1. Updates the system
#   2. Installs Docker (ROS2 runs in a container, keeps the Pi OS clean)
#   3. Installs basic tools (git, python, serial)
#   4. Sets up udev rules for rover USB connections
#   5. Creates the COVEN workspace directory
#
# Why Docker for the dock?
#   ROS2 Jazzy is built for Ubuntu 24.04. Pi OS is based on Debian.
#   Rather than fighting package conflicts, we run ROS2 in a container
#   that has the right Ubuntu base. Your code mounts into it, so you
#   edit on the Pi and it runs inside the container. Same as your dev
#   machine setup.
#
# =============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== COVEN Dock Setup (Pi 4) ===${NC}"
echo ""

# --- Check we're on the right hardware ---
if [ "$(uname -m)" != "aarch64" ]; then
    echo -e "${RED}This doesn't look like a 64-bit Pi. Got: $(uname -m)${NC}"
    exit 1
fi

# =============================================================================
# Step 1: System update
# =============================================================================
echo -e "${YELLOW}[1/5] Updating system packages...${NC}"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y -o Dpkg::Options::="--force-confold"

# =============================================================================
# Step 2: Install Docker
# =============================================================================
echo -e "${YELLOW}[2/5] Installing Docker...${NC}"

if command -v docker &>/dev/null; then
    echo -e "${GREEN}Docker already installed.${NC}"
else
    # Install Docker using the official convenience script
    # This auto-detects Pi OS and installs the right version
    curl -fsSL https://get.docker.com | sudo sh

    # Let the 'coven' user run docker without sudo
    sudo usermod -aG docker "$USER"

    echo -e "${GREEN}Docker installed. You'll need to log out and back in for${NC}"
    echo -e "${GREEN}group membership to take effect (or run: newgrp docker).${NC}"
fi

# Install docker compose plugin if not present
if ! docker compose version &>/dev/null 2>&1; then
    sudo apt-get install -y docker-compose-plugin
fi

# =============================================================================
# Step 3: Install system tools
# =============================================================================
echo -e "${YELLOW}[3/5] Installing tools...${NC}"

sudo apt-get install -y \
    git \
    python3-pip \
    python3-serial \
    i2c-tools \
    screen \
    htop \
    tree

# =============================================================================
# Step 4: Udev rules for rover USB connections
# =============================================================================
echo -e "${YELLOW}[4/5] Setting up udev rules for rover connections...${NC}"

# When a rover is plugged into the dock via USB, it appears as a serial
# device (thanks to g_serial on the rover side). This rule gives each
# rover a predictable device name based on which USB port it's plugged into.
#
# Without this: rovers show up as /dev/ttyACM0, /dev/ttyACM1, etc.
#   but which number is which depends on plug order.
#
# With this: you can assign specific USB ports to specific rovers.
#   For now we just make sure the permissions are right so Docker
#   can talk to them without root.

sudo tee /etc/udev/rules.d/99-coven-rovers.rules > /dev/null <<'EOF'
# COVEN rover USB gadget serial connections
# Pi Zero 2W in USB gadget mode appears as CDC ACM device
SUBSYSTEM=="tty", ATTRS{idVendor}=="0525", ATTRS{idProduct}=="a4a7", MODE="0666", GROUP="dialout"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "${GREEN}Udev rules installed. Rovers will be accessible as /dev/ttyACM*.${NC}"

# =============================================================================
# Step 5: Create workspace and clone code
# =============================================================================
echo -e "${YELLOW}[5/5] Setting up workspace...${NC}"

WORKSPACE="$HOME/coven"
mkdir -p "$WORKSPACE"

# If the code repo isn't cloned yet, tell the user how
if [ ! -d "$WORKSPACE/COVEN-CORE" ]; then
    echo -e "${YELLOW}Workspace created at: $WORKSPACE${NC}"
    echo ""
    echo "Next step: get your code onto this Pi. Options:"
    echo ""
    echo "  Option A — Clone from git (if you've pushed to a remote):"
    echo "    cd $WORKSPACE && git clone <your-repo-url> COVEN-CORE"
    echo ""
    echo "  Option B — Copy from your laptop:"
    echo "    scp -r /path/to/COVEN-CORE coven@$(hostname).local:$WORKSPACE/"
    echo ""
    echo "  Option C — rsync (better for repeated syncs during development):"
    echo "    rsync -avz --exclude='.cache' --exclude='target' \\"
    echo "      /path/to/COVEN-CORE/ coven@$(hostname).local:$WORKSPACE/COVEN-CORE/"
    echo ""
else
    echo -e "${GREEN}Code already present at $WORKSPACE/COVEN-CORE${NC}"
fi

# =============================================================================
# Done
# =============================================================================
echo ""
echo -e "${GREEN}=== Dock setup complete! ===${NC}"
echo ""
echo "What you have now:"
echo "  - Docker installed (ROS2 Jazzy runs in a container)"
echo "  - Serial/I2C tools for hardware debugging"
echo "  - Udev rules for rover USB connections"
echo ""
echo "To start the dock software:"
echo "  cd $WORKSPACE/COVEN-CORE/coven_core/docker"
echo "  ./run.sh build    # First time: builds the ROS2 container (~10 min)"
echo "  ./run.sh dock     # Launches the dock nodes"
echo ""
echo "To check if a rover is connected:"
echo "  ls /dev/ttyACM*   # Should show up when a rover is USB-plugged"
echo ""
if ! groups | grep -q docker; then
    echo -e "${YELLOW}IMPORTANT: Log out and back in for Docker permissions to work.${NC}"
    echo "  Or run: newgrp docker"
fi
