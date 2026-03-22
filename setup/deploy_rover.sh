#!/bin/bash
# =============================================================================
# deploy_rover.sh — Cross-compile and deploy rover firmware to a Pi Zero 2W
# =============================================================================
#
# Run this FROM YOUR DEV MACHINE (not on the Pi).
#
# What this does:
#   1. Cross-compiles the Rust rover firmware for ARM64 (aarch64)
#   2. Copies the binary to the Pi over SSH
#   3. Restarts the service on the Pi
#
# Usage:
#   ./deploy_rover.sh <hostname>
#
# Examples:
#   ./deploy_rover.sh morgan-le-fay     # Deploy to rover "morgan-le-fay"
#   ./deploy_rover.sh kiki      # Deploy to rover "kiki"
#
# Prerequisites (on your dev machine):
#   - Rust installed (rustup)
#   - Cross-compilation target: rustup target add aarch64-unknown-linux-gnu
#   - ARM64 linker: sudo dnf install gcc-aarch64-linux-gnu  (Fedora)
#                   sudo apt install gcc-aarch64-linux-gnu   (Ubuntu/Debian)
#
# =============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

if [ $# -lt 1 ]; then
    echo -e "${RED}Usage: ./deploy_rover.sh <hostname>${NC}"
    echo "  e.g., ./deploy_rover.sh morgan-le-fay"
    exit 1
fi

TARGET_HOST="$1"
PI_USER="${COVEN_PI_USER:-coven}"
ROVER_DIR="$(dirname "$0")/../coven_core/rover"

echo -e "${CYAN}=== Deploy Rover Firmware to $TARGET_HOST ===${NC}"
echo ""

# --- Check prerequisites ---
if ! command -v cargo &>/dev/null; then
    echo -e "${RED}Rust/Cargo not found. Install from: https://rustup.rs${NC}"
    exit 1
fi

if ! rustup target list --installed | grep -q "aarch64-unknown-linux-gnu"; then
    echo -e "${YELLOW}Adding aarch64 target...${NC}"
    rustup target add aarch64-unknown-linux-gnu
fi

if ! command -v aarch64-linux-gnu-gcc &>/dev/null; then
    echo -e "${RED}ARM64 cross-compiler not found.${NC}"
    echo "Install it:"
    echo "  Fedora: sudo dnf install gcc-aarch64-linux-gnu"
    echo "  Ubuntu: sudo apt install gcc-aarch64-linux-gnu"
    exit 1
fi

# --- Build ---
echo -e "${YELLOW}[1/3] Cross-compiling rover firmware...${NC}"
cd "$ROVER_DIR"
cargo build --release --target aarch64-unknown-linux-gnu

BINARY="target/aarch64-unknown-linux-gnu/release/coven-rover"
if [ ! -f "$BINARY" ]; then
    echo -e "${RED}Build succeeded but binary not found at: $BINARY${NC}"
    exit 1
fi

SIZE=$(du -h "$BINARY" | cut -f1)
echo -e "${GREEN}Build complete. Binary size: $SIZE${NC}"

# --- Deploy ---
echo -e "${YELLOW}[2/3] Copying binary to $TARGET_HOST...${NC}"
scp "$BINARY" "${PI_USER}@${TARGET_HOST}.local:/tmp/coven-rover"
ssh "${PI_USER}@${TARGET_HOST}.local" "sudo mv /tmp/coven-rover /usr/local/bin/coven-rover && sudo chmod +x /usr/local/bin/coven-rover"

echo -e "${GREEN}Binary installed.${NC}"

# --- Restart service ---
echo -e "${YELLOW}[3/3] Restarting rover service...${NC}"
ssh "${PI_USER}@${TARGET_HOST}.local" "sudo systemctl restart coven-rover 2>/dev/null || echo 'Service not enabled yet — run: sudo systemctl enable --now coven-rover'"

echo ""
echo -e "${GREEN}=== Deployed to $TARGET_HOST ===${NC}"
echo ""
echo "Check status: ssh ${PI_USER}@${TARGET_HOST}.local 'sudo journalctl -u coven-rover -n 20'"
