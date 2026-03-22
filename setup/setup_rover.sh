#!/bin/bash
# =============================================================================
# setup_rover.sh — Set up a Pi Zero 2W as a COVEN rover
# =============================================================================
#
# Run this ON the Pi Zero 2W after first boot:
#   ssh coven@morgan.local    (or whatever hostname you gave it)
#   # then run this script
#
# What this does:
#   1. Updates the system
#   2. Installs I2C and serial tools
#   3. Verifies USB gadget serial is working
#   4. Verifies I2C is enabled (for the battery ADC)
#   5. Creates the config directory and installs rover.toml
#   6. Installs the pre-built rover binary (you cross-compile on your laptop)
#
# What this does NOT do:
#   - Install ROS2 (rovers don't run ROS2 — they run Rust firmware)
#   - Install Rust (you cross-compile on your laptop, not on the Pi Zero)
#
# The Pi Zero 2W has 512MB RAM. That's enough to run the rover firmware
# but NOT enough to compile Rust. You build on your laptop and copy the
# binary over. That's what the deploy_rover.sh script is for.
#
# =============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== COVEN Rover Setup (Pi Zero 2W) ===${NC}"
echo ""

# --- Check we're on the right hardware ---
if [ "$(uname -m)" != "aarch64" ]; then
    echo -e "${RED}This doesn't look like a 64-bit Pi. Got: $(uname -m)${NC}"
    exit 1
fi

HOSTNAME=$(hostname)
echo -e "Hostname: ${GREEN}$HOSTNAME${NC}"

# =============================================================================
# Step 1: System update
# =============================================================================
echo -e "${YELLOW}[1/6] Updating system packages...${NC}"
echo "(This can be slow on the Pi Zero — it's a single-core ARM doing apt.)"
echo ""
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y -o Dpkg::Options::="--force-confold"

# =============================================================================
# Step 2: Install tools
# =============================================================================
echo -e "${YELLOW}[2/6] Installing tools...${NC}"

sudo apt-get install -y \
    i2c-tools \
    python3-pip \
    python3-smbus \
    screen \
    htop

# =============================================================================
# Step 3: Verify USB gadget serial
# =============================================================================
echo -e "${YELLOW}[3/6] Checking USB gadget serial...${NC}"

# The flash script should have set this up, but let's verify
GADGET_OK=true

if ! grep -q "dwc2" /boot/firmware/config.txt 2>/dev/null && \
   ! grep -q "dwc2" /boot/config.txt 2>/dev/null; then
    echo -e "${RED}dwc2 overlay not found in config.txt${NC}"
    GADGET_OK=false
fi

if ! grep -q "dwc2" /etc/modules; then
    echo -e "${RED}dwc2 not in /etc/modules${NC}"
    GADGET_OK=false
fi

if ! grep -q "g_serial" /etc/modules; then
    echo -e "${RED}g_serial not in /etc/modules${NC}"
    GADGET_OK=false
fi

if $GADGET_OK; then
    echo -e "${GREEN}USB gadget serial is configured.${NC}"
    echo "When you plug this rover into the dock via USB:"
    echo "  - Dock sees: /dev/ttyACM0 (serial port to talk to this rover)"
    echo "  - Rover sees: /dev/ttyGS0 (serial port to talk to the dock)"
else
    echo -e "${RED}USB gadget serial is NOT fully configured.${NC}"
    echo "The flash script should have done this. You may need to re-flash"
    echo "or manually add dwc2/g_serial to /etc/modules and config.txt."
fi

# =============================================================================
# Step 4: Verify I2C
# =============================================================================
echo -e "${YELLOW}[4/6] Checking I2C...${NC}"

if sudo i2cdetect -y 1 > /dev/null 2>&1; then
    echo -e "${GREEN}I2C bus 1 is working.${NC}"

    # Check for ADS1015 at address 0x48
    if sudo i2cdetect -y 1 | grep -q "48"; then
        echo -e "${GREEN}ADS1015 ADC found at 0x48${NC}"
    else
        echo -e "${YELLOW}ADS1015 not detected at 0x48 (normal if not wired yet).${NC}"
    fi
else
    echo -e "${RED}I2C is not working. Check that dtparam=i2c_arm=on is in config.txt${NC}"
fi

# =============================================================================
# Step 5: Create config directory and install rover.toml
# =============================================================================
echo -e "${YELLOW}[5/6] Setting up COVEN config...${NC}"

sudo mkdir -p /etc/coven
sudo mkdir -p /var/lib/coven/data    # Sensor data storage

# Install a default rover.toml
# You'll want to customize rover_id and coven_name for each rover
if [ ! -f /etc/coven/rover.toml ]; then
    sudo tee /etc/coven/rover.toml > /dev/null <<'TOMLEOF'
# COVEN Rover Configuration
# Edit rover_id and coven_name to match this rover's identity

# Rover identity — each rover gets a unique witch name
rover_id = "CHANGE_ME"
coven_name = "The_Bene_Gesserit"

# Dock UART
# MVP: USB gadget serial (/dev/ttyGS0) — hand-plugged USB cable
# Production: GPIO UART (/dev/ttyAMA0) — 9-pin magnetic connector
[dock_uart]
port = "/dev/ttyGS0"
baud_rate = 115200

[hardware.motors]
# TB6612FNG pin assignments (BCM numbering)
left_pwm = 12
left_in1 = 5
left_in2 = 6
right_pwm = 13
right_in1 = 16
right_in2 = 26
standby = 17

pwm_frequency = 1000.0
wheel_base = 0.298     # meters (CubeRover 2U)
wheel_radius = 0.0325  # meters (65mm diameter wheels)
max_rpm = 130.0

[hardware.encoders]
# Quadrature encoder pins (BCM numbering)
left_a = 23
left_b = 24
right_a = 27
right_b = 22

pulses_per_rev = 312   # JGA25-371, 26:1 gearbox, 12 PPR

[hardware.lidar]
port = "/dev/rplidar"
baud_rate = 460800
scan_rate = 5.5
range_min = 0.15
range_max = 12.0

[hardware.battery]
adc_address = 0x48
divider_ratio = 11.0
full_voltage = 12.6
empty_voltage = 9.6

[timing]
control_rate = 20.0
heartbeat_rate = 1.0
odom_rate = 50.0
cmd_timeout = 0.5
battery_check_interval = 2.0

[navigation]
k_att = 1.0
k_rep = 0.8
d_influence = 1.5
d_safe = 0.25
goal_tolerance = 0.1
max_linear = 0.3
max_angular = 1.5
k_heading = 2.0
low_battery_threshold = 15.0
mission_timeout_factor = 5.0
TOMLEOF

    echo -e "${YELLOW}Default rover.toml installed at /etc/coven/rover.toml${NC}"
    echo -e "${YELLOW}IMPORTANT: Edit rover_id to this rover's witch name!${NC}"
    echo "  sudo nano /etc/coven/rover.toml"
else
    echo -e "${GREEN}rover.toml already exists at /etc/coven/rover.toml${NC}"
fi

# =============================================================================
# Step 6: Install directory for the rover binary
# =============================================================================
echo -e "${YELLOW}[6/6] Setting up binary install location...${NC}"

sudo mkdir -p /usr/local/bin

# Create a systemd service so the rover firmware starts on boot
sudo tee /etc/systemd/system/coven-rover.service > /dev/null <<'SVCEOF'
[Unit]
Description=COVEN Rover Firmware
After=network.target
Wants=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/coven-rover --config /etc/coven/rover.toml
Restart=on-failure
RestartSec=5
User=root
# Root needed for GPIO access via /dev/mem (rppal)

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=coven-rover

[Install]
WantedBy=multi-user.target
SVCEOF

# Don't enable yet — binary isn't installed
echo -e "${GREEN}Systemd service created (not enabled until binary is deployed).${NC}"

# =============================================================================
# Done
# =============================================================================
echo ""
echo -e "${GREEN}=== Rover setup complete! ===${NC}"
echo ""
echo "This Pi is ready to receive the rover firmware."
echo ""
echo "Next steps:"
echo ""
echo "  1. Edit the rover identity:"
echo "     sudo nano /etc/coven/rover.toml"
echo "     Change rover_id from CHANGE_ME to this rover's witch name"
echo ""
echo "  2. Wire up your hardware (motors, ADC, LiDAR)"
echo ""
echo "  3. Cross-compile and deploy the firmware (from your laptop):"
echo "     cd COVEN-CORE/setup"
echo "     ./deploy_rover.sh $HOSTNAME"
echo ""
echo "  4. After deploying, start the service:"
echo "     sudo systemctl enable coven-rover"
echo "     sudo systemctl start coven-rover"
echo ""
echo "  5. Check if it's running:"
echo "     sudo journalctl -u coven-rover -f"
