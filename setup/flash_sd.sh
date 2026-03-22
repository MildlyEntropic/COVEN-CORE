#!/bin/bash
# =============================================================================
# flash_sd.sh — Flash and preconfigure an SD card for a COVEN Pi
# =============================================================================
#
# What this does:
#   1. Downloads Raspberry Pi OS Lite (64-bit) if you don't have it already
#   2. Writes it to your SD card
#   3. Pre-configures WiFi, SSH, and hostname so the Pi is reachable on first boot
#
# Usage:
#   ./flash_sd.sh <role> <hostname>
#
#   role:     "dock" or "rover"
#   hostname: whatever you want to call it (e.g., "bene-gesserit", "morgan-le-fay", "kiki")
#
# Examples:
#   ./flash_sd.sh dock bene-gesserit       # Flash the Pi 4 dock card
#   ./flash_sd.sh rover morgan-le-fay          # Flash rover 1 (Morgan Le Fay)
#   ./flash_sd.sh rover kiki           # Flash rover 2 (Circe)
#
# IMPORTANT: This script needs sudo because writing to raw block devices
#            (your SD card) requires root. It will ask for your password.
#
# =============================================================================

set -euo pipefail

# --- cd to script's own directory so paths work no matter where you run from ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# --- Validate arguments ---
if [ $# -lt 2 ]; then
    echo -e "${RED}Usage: ./flash_sd.sh <role> <hostname>${NC}"
    echo "  role:     dock | rover"
    echo "  hostname: e.g., bene-gesserit, morgan-le-fay, kiki"
    exit 1
fi

ROLE="$1"
HOSTNAME="$2"

if [[ "$ROLE" != "dock" && "$ROLE" != "rover" ]]; then
    echo -e "${RED}Role must be 'dock' or 'rover', got: $ROLE${NC}"
    exit 1
fi

# --- Configuration ---
# You'll need to set these for your WiFi network
WIFI_SSID="${COVEN_WIFI_SSID:-Secret Rebel Base}"
WIFI_PASS="${COVEN_WIFI_PASS:-Y^5tR\$3eW@1q}"
WIFI_COUNTRY="${COVEN_WIFI_COUNTRY:-US}"

# Pi user credentials (change the password!)
PI_USER="coven"
PI_PASS="${COVEN_PI_PASS:-1123}"

# Where to cache the downloaded image
CACHE_DIR="$(dirname "$0")/.cache"
IMAGE_URL="https://downloads.raspberrypi.com/raspios_lite_arm64/images/raspios_lite_arm64-2025-05-13/2025-05-13-raspios-bookworm-arm64-lite.img.xz"
IMAGE_FILE="$CACHE_DIR/raspios-bookworm-arm64-lite.img.xz"
IMAGE_UNCOMPRESSED="$CACHE_DIR/raspios-bookworm-arm64-lite.img"

# =============================================================================
# Step 1: Find the SD card
# =============================================================================
echo -e "${CYAN}=== COVEN SD Card Flasher ===${NC}"
echo -e "Role: ${GREEN}$ROLE${NC}  Hostname: ${GREEN}$HOSTNAME${NC}"
echo ""

# List removable block devices (SD cards, USB drives — NOT your hard drive)
echo -e "${YELLOW}Looking for SD cards...${NC}"
echo ""

# Show only removable devices to avoid nuking your system drive
REMOVABLE_DEVS=()
for dev in /sys/block/sd* /sys/block/mmcblk*; do
    [ -e "$dev" ] || continue
    devname=$(basename "$dev")
    removable=$(cat "$dev/removable" 2>/dev/null || echo "0")
    # mmcblk devices are always SD cards; sd* devices check removable flag
    if [[ "$devname" == mmcblk* ]] || [[ "$removable" == "1" ]]; then
        size_sectors=$(cat "$dev/size" 2>/dev/null || echo "0")
        size_gb=$(( size_sectors * 512 / 1073741824 ))
        echo "  /dev/$devname  —  ${size_gb}GB"
        REMOVABLE_DEVS+=("/dev/$devname")
    fi
done

if [ ${#REMOVABLE_DEVS[@]} -eq 0 ]; then
    echo -e "${RED}No removable drives found. Is your SD card plugged in?${NC}"
    echo "If you're using a USB card reader, it might show up as /dev/sdX."
    echo "Run 'lsblk' to see all drives, then set SD_DEVICE manually:"
    echo "  SD_DEVICE=/dev/sdX ./flash_sd.sh $ROLE $HOSTNAME"
    exit 1
fi

# Let the user pick if there's more than one, or auto-select if just one
if [ -n "${SD_DEVICE:-}" ]; then
    TARGET="$SD_DEVICE"
    echo -e "Using SD_DEVICE override: ${GREEN}$TARGET${NC}"
elif [ ${#REMOVABLE_DEVS[@]} -eq 1 ]; then
    TARGET="${REMOVABLE_DEVS[0]}"
    echo -e "Found one removable device: ${GREEN}$TARGET${NC}"
else
    echo ""
    echo "Multiple removable devices found. Which one is the SD card?"
    select TARGET in "${REMOVABLE_DEVS[@]}"; do
        [ -n "$TARGET" ] && break
    done
fi

echo ""
echo -e "${RED}WARNING: This will ERASE everything on $TARGET${NC}"
echo -e "Make sure this is your SD card and not something important!"
read -p "Type 'yes' to continue: " CONFIRM
if [ "$CONFIRM" != "yes" ]; then
    echo "Aborted."
    exit 1
fi

# =============================================================================
# Step 2: Download Pi OS (if needed)
# =============================================================================
mkdir -p "$CACHE_DIR"

if [ -f "$IMAGE_UNCOMPRESSED" ]; then
    echo -e "${GREEN}Using cached image: $IMAGE_UNCOMPRESSED${NC}"
elif [ -f "$IMAGE_FILE" ]; then
    echo -e "${YELLOW}Decompressing cached image...${NC}"
    xz -dk "$IMAGE_FILE"
else
    echo -e "${YELLOW}Downloading Raspberry Pi OS Lite (64-bit)...${NC}"
    echo "This is ~400MB, might take a few minutes."
    echo ""

    # If the hardcoded URL is stale, fall back to rpi-imager or manual download
    if ! curl -fL --progress-bar -o "$IMAGE_FILE" "$IMAGE_URL"; then
        echo -e "${RED}Download failed. The URL may be outdated.${NC}"
        echo ""
        echo "Download manually from: https://www.raspberrypi.com/software/operating-systems/"
        echo "Pick: Raspberry Pi OS Lite (64-bit, Bookworm)"
        echo "Save the .img file to: $IMAGE_UNCOMPRESSED"
        echo "Then re-run this script."
        rm -f "$IMAGE_FILE"
        exit 1
    fi

    echo -e "${YELLOW}Decompressing...${NC}"
    xz -dk "$IMAGE_FILE"
fi

# =============================================================================
# Step 3: Write the image to the SD card
# =============================================================================
echo -e "${YELLOW}Writing image to $TARGET...${NC}"
echo "This takes a few minutes. Don't unplug the card."
echo ""

# Unmount any partitions that might be auto-mounted
sudo umount "${TARGET}"* 2>/dev/null || true

# Write the image
# dd writes the raw disk image byte-for-byte to the SD card
# bs=4M = write in 4MB chunks (faster than default 512 bytes)
# status=progress = show a progress bar
sudo dd if="$IMAGE_UNCOMPRESSED" of="$TARGET" bs=4M status=progress conv=fsync

# Make sure everything is flushed to disk
sync

echo -e "${GREEN}Image written.${NC}"

# =============================================================================
# Step 4: Configure the card (WiFi, SSH, hostname, user)
# =============================================================================
echo -e "${YELLOW}Configuring first-boot settings...${NC}"

# The SD card now has two partitions:
#   - boot (FAT32) — firmware, config.txt, kernel
#   - rootfs (ext4) — the actual Linux filesystem
#
# We need to mount both to drop config files in.

# Figure out partition naming (mmcblk0p1 vs sda1)
if [[ "$TARGET" == *mmcblk* ]]; then
    PART_BOOT="${TARGET}p1"
    PART_ROOT="${TARGET}p2"
else
    PART_BOOT="${TARGET}1"
    PART_ROOT="${TARGET}2"
fi

# Re-read partition table so the OS sees the new partitions
sudo partprobe "$TARGET" 2>/dev/null || true
sleep 3

# Unmount anything the desktop auto-mounted, then mount to known locations
sudo umount "$PART_BOOT" 2>/dev/null || true
sudo umount "$PART_ROOT" 2>/dev/null || true
sleep 1

MOUNT_BOOT=$(mktemp -d)
MOUNT_ROOT=$(mktemp -d)
sudo mount "$PART_BOOT" "$MOUNT_BOOT"
sudo mount "$PART_ROOT" "$MOUNT_ROOT"

# --- Enable SSH on first boot ---
# An empty file called "ssh" in the boot partition tells Pi OS to start sshd
sudo touch "$MOUNT_BOOT/ssh"

# --- Create user account ---
# Pi OS no longer has a default "pi" user. We create one via userconf.txt.
# Format: username:encrypted-password
ENCRYPTED_PASS=$(openssl passwd -6 "$PI_PASS")
echo "${PI_USER}:${ENCRYPTED_PASS}" | sudo tee "$MOUNT_BOOT/userconf.txt" > /dev/null

# --- Set hostname ---
echo "$HOSTNAME" | sudo tee "$MOUNT_ROOT/etc/hostname" > /dev/null
sudo sed -i "s/127.0.1.1.*/127.0.1.1\t$HOSTNAME/" "$MOUNT_ROOT/etc/hosts"

# --- Configure WiFi ---
if [ -n "$WIFI_SSID" ]; then
    # NetworkManager config (Pi OS Bookworm uses NetworkManager, not wpa_supplicant)
    sudo mkdir -p "$MOUNT_ROOT/etc/NetworkManager/system-connections"
    # Write the connection file manually to avoid heredoc escaping issues
    # with special characters in passwords
    {
        echo "[connection]"
        echo "id=coven-wifi"
        echo "type=wifi"
        echo "autoconnect=true"
        echo ""
        echo "[wifi]"
        echo "ssid=${WIFI_SSID}"
        echo "mode=infrastructure"
        echo ""
        echo "[wifi-security]"
        echo "key-mgmt=wpa-psk"
        echo "psk=${WIFI_PASS}"
        echo ""
        echo "[ipv4]"
        echo "method=auto"
        echo ""
        echo "[ipv6]"
        echo "method=auto"
    } | sudo tee "$MOUNT_ROOT/etc/NetworkManager/system-connections/coven-wifi.nmconnection" > /dev/null
    sudo chmod 600 "$MOUNT_ROOT/etc/NetworkManager/system-connections/coven-wifi.nmconnection"
    # Set WiFi regulatory country — Pi won't scan without this
    sudo sed -i '/^#.*REGDOMAIN/d' "$MOUNT_ROOT/etc/default/crda" 2>/dev/null || true
    echo "REGDOMAIN=$WIFI_COUNTRY" | sudo tee "$MOUNT_ROOT/etc/default/crda" > /dev/null 2>/dev/null || true

    # Also set via raspi-config's method (wpa_supplicant country)
    sudo tee "$MOUNT_BOOT/wpa_supplicant.conf" > /dev/null <<WPAEOF
country=$WIFI_COUNTRY
WPAEOF

    # Unblock WiFi by default
    sudo mkdir -p "$MOUNT_ROOT/var/lib/systemd/rfkill"
    echo "0" | sudo tee "$MOUNT_ROOT/var/lib/systemd/rfkill/platform-3f300000.mmcnr:wlan" > /dev/null 2>/dev/null || true
    echo "0" | sudo tee "$MOUNT_ROOT/var/lib/systemd/rfkill/platform-fe300000.mmcnr:wlan" > /dev/null 2>/dev/null || true

    echo -e "${GREEN}WiFi configured: $WIFI_SSID (country: $WIFI_COUNTRY)${NC}"
else
    echo -e "${YELLOW}No WiFi configured. Set COVEN_WIFI_SSID and COVEN_WIFI_PASS env vars.${NC}"
    echo "You can also configure WiFi after boot with: sudo nmtui"
fi

# --- Enable USB gadget serial (rovers only) ---
# This makes the Pi Zero 2W appear as a serial device when plugged into
# the dock via USB. The dock sees it as /dev/ttyACM0, the rover sees /dev/ttyGS0.
if [ "$ROLE" = "rover" ]; then
    # Add dwc2 overlay to config.txt (tells the USB hardware to act as a device)
    # IMPORTANT: must be just "dtoverlay=dwc2" — NOT "dtoverlay=dwc2,dr_mode=host"
    # dr_mode=host prevents gadget mode from working
    sudo sed -i '/dtoverlay=dwc2/d' "$MOUNT_BOOT/config.txt"
    echo "dtoverlay=dwc2" | sudo tee -a "$MOUNT_BOOT/config.txt" > /dev/null

    # Load the dwc2 kernel module at boot
    if ! grep -q "dwc2" "$MOUNT_ROOT/etc/modules"; then
        echo "dwc2" | sudo tee -a "$MOUNT_ROOT/etc/modules" > /dev/null
    fi

    # Load the USB serial gadget module at boot
    # g_serial makes the Pi show up as a serial port when USB-connected
    if ! grep -q "g_serial" "$MOUNT_ROOT/etc/modules"; then
        echo "g_serial" | sudo tee -a "$MOUNT_ROOT/etc/modules" > /dev/null
    fi

    echo -e "${GREEN}USB gadget serial enabled (rover MVP mode)${NC}"
fi

# --- Enable I2C and SPI ---
# I2C is needed for the ADS1015 ADC (battery monitoring)
# SPI is available for future sensors
# Pi OS has these commented out by default, so uncomment them.
# If they're not present at all, append them.
if grep -q "#dtparam=i2c_arm=on" "$MOUNT_BOOT/config.txt"; then
    sudo sed -i 's/#dtparam=i2c_arm=on/dtparam=i2c_arm=on/' "$MOUNT_BOOT/config.txt"
elif ! grep -q "dtparam=i2c_arm=on" "$MOUNT_BOOT/config.txt"; then
    echo "dtparam=i2c_arm=on" | sudo tee -a "$MOUNT_BOOT/config.txt" > /dev/null
fi
if grep -q "#dtparam=spi=on" "$MOUNT_BOOT/config.txt"; then
    sudo sed -i 's/#dtparam=spi=on/dtparam=spi=on/' "$MOUNT_BOOT/config.txt"
elif ! grep -q "dtparam=spi=on" "$MOUNT_BOOT/config.txt"; then
    echo "dtparam=spi=on" | sudo tee -a "$MOUNT_BOOT/config.txt" > /dev/null
fi

# --- Create udev rule for RPLIDAR ---
# When you plug in the RPLIDAR, it shows up as /dev/ttyUSB0 or ttyUSB1 or
# whatever. This rule gives it a stable name: /dev/rplidar
# That way rover.toml can always point to /dev/rplidar and it just works.
if [ "$ROLE" = "rover" ]; then
    sudo tee "$MOUNT_ROOT/etc/udev/rules.d/99-rplidar.rules" > /dev/null <<'UDEVEOF'
# RPLIDAR C1 — Silicon Labs CP2102 USB-UART
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"
UDEVEOF
    echo -e "${GREEN}RPLIDAR udev rule installed (/dev/rplidar)${NC}"
fi

# --- Fix WiFi firmware country code (Pi Zero 2W / BCM43436) ---
# The Broadcom WiFi chip has its own firmware-level country code that overrides
# iw reg set and everything else. Default is "ALL" (country 99) which restricts
# channels and prevents nmcli from finding networks.
for fw_file in "$MOUNT_ROOT"/lib/firmware/brcm/brcmfmac43436*-sdio*.txt; do
    if [ -f "$fw_file" ]; then
        sudo sed -i 's/ccode=ALL/ccode=US/' "$fw_file"
    fi
done

# --- Create WiFi unblock service ---
# Pi OS soft-blocks WiFi by default and rfkill state resets on every boot.
# This service unblocks it before NetworkManager starts.
sudo tee "$MOUNT_ROOT/etc/systemd/system/unblock-wifi.service" > /dev/null <<'SVCEOF'
[Unit]
Description=Unblock WiFi
Before=NetworkManager.service
After=systemd-rfkill.service

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock wifi
ExecStartPost=/usr/sbin/iw reg set US

[Install]
WantedBy=multi-user.target
SVCEOF
sudo ln -sf /etc/systemd/system/unblock-wifi.service \
    "$MOUNT_ROOT/etc/systemd/system/multi-user.target.wants/unblock-wifi.service"

# --- Create rc.local WiFi fallback ---
# Belt-and-suspenders: if NetworkManager doesn't connect on its own,
# this script retries with nmcli and falls back to wpa_supplicant.
sudo tee "$MOUNT_ROOT/etc/rc.local" > /dev/null <<'RCEOF'
#!/bin/bash
exec > /var/log/wifi-debug.log 2>&1
echo "rc.local starting at $(date)"

rfkill unblock wifi
sleep 2
ip link set wlan0 up
sleep 3

# Restart NM to pick up country code
systemctl restart NetworkManager
sleep 5

nmcli device wifi rescan ifname wlan0 2>&1
sleep 3

for i in $(seq 1 5); do
    echo "Connect attempt $i..."
    nmcli device wifi connect "Secret Rebel Base" password 'Y^5tR$3eW@1q' ifname wlan0 2>&1 && break
    sleep 10
done

# Fallback: wpa_supplicant directly
if ! ip addr show wlan0 | grep -q "inet "; then
    echo "nmcli failed, trying wpa_supplicant..."
    killall wpa_supplicant 2>/dev/null
    sleep 1
    cat > /tmp/wpa.conf <<WPAEOF
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
network={
    ssid="Secret Rebel Base"
    psk="Y^5tR\$3eW@1q"
    key_mgmt=WPA-PSK
}
WPAEOF
    wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf 2>&1
    sleep 5
    dhclient wlan0 2>&1 || dhcpcd wlan0 2>&1
fi

echo "=== final state ==="
ip addr show wlan0
echo "rc.local done at $(date)"
exit 0
RCEOF
sudo chmod +x "$MOUNT_ROOT/etc/rc.local"
sudo ln -sf /lib/systemd/system/rc-local.service \
    "$MOUNT_ROOT/etc/systemd/system/multi-user.target.wants/rc-local.service"

# --- Enable persistent journal for debugging ---
sudo mkdir -p "$MOUNT_ROOT/var/log/journal"

# --- Drop a marker file so setup scripts know the role ---
echo "$ROLE" | sudo tee "$MOUNT_ROOT/etc/coven-role" > /dev/null

# --- Clean up ---
sudo umount "$MOUNT_BOOT"
sudo umount "$MOUNT_ROOT"
rmdir "$MOUNT_BOOT" "$MOUNT_ROOT"
sync

echo ""
echo -e "${GREEN}=== Done! ===${NC}"
echo ""
echo "Card is ready. Here's what happens next:"
echo ""
echo "  1. Put the card in your ${ROLE} Pi"
echo "  2. Power it on and wait ~60 seconds for first boot"
echo "  3. Find it on the network:  ssh ${PI_USER}@${HOSTNAME}.local"
echo "     Password: ${PI_PASS}"
echo "  4. Run the setup script:    ./setup_${ROLE}.sh"
echo ""
if [ -z "$WIFI_SSID" ]; then
    echo -e "${YELLOW}  NOTE: WiFi not configured. Connect ethernet or run 'sudo nmtui' on the Pi.${NC}"
fi
