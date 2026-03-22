# COVEN Hardware Setup Guide

You have 3 SD cards and 3 Pis. Here's what goes where and how to get them
running.

## The Cast

| Pi | Role | Hostname | What it does |
|----|------|----------|-------------|
| Raspberry Pi 4 (4GB) | Dock | `bene-gesserit` | Runs ROS2 in Docker. Talks to rovers via USB serial. Runs SLAM on their sensor data. Assigns new missions. The brain. |
| Pi Zero 2W #1 | Rover | `morgan-le-fay` | Runs Rust firmware. Drives around autonomously. Collects LiDAR/odometry data. Returns to dock to upload. The legs. |
| Pi Zero 2W #2 | Rover | `kiki` | Same as above, different witch name. |

## What You Need

- Your dev machine (the Fedora tower you're on right now)
- 3x 128GB microSD cards (SanDisk Ultra Plus, A1/U1/V10)
- A USB SD card reader (if your tower doesn't have a built-in SD slot)
- Your WiFi network name and password
- The 3 Pis

## Step 1: Flash the SD Cards

You're going to write an operating system onto each SD card, then
preconfigure it so the Pi connects to WiFi and accepts SSH on first boot.
Do this one card at a time from your dev machine.

### 1a. Flash each card

WiFi credentials and Pi login password are already baked into
`flash_sd.sh` — no setup needed. For each of the 3 cards:

1. Put the microSD card into your card reader
2. Run the flash command for that card's role:

```bash
# Card 1 — the dock (goes in the Pi 4)
./flash_sd.sh dock bene-gesserit

# Card 2 — rover 1 (goes in a Pi Zero 2W)
./flash_sd.sh rover morgan-le-fay

# Card 3 — rover 2 (goes in the other Pi Zero 2W)
./flash_sd.sh rover kiki
```

4. The script will find the SD card, show you which device it found, and
   ask you to confirm before writing (so you don't accidentally nuke
   something important)
5. It downloads Pi OS the first time (~400MB), then caches it for the
   other two cards
6. After writing, it mounts the card and drops in your WiFi config,
   SSH keys, hostname, and hardware settings
7. When it says "Done", pull the card out of the reader

What each card gets configured with:
- Raspberry Pi OS Lite (64-bit) — command line only, no desktop
- Your WiFi credentials — so it connects to your network on first boot
- SSH enabled — so you can log in remotely from your tower
- A user account (`coven` / your password)
- A hostname — so you can reach it as `bene-gesserit.local`, `morgan.local`, etc.
- For rover cards: USB gadget serial enabled + RPLIDAR udev rule
- For all cards: I2C enabled (needed for the battery ADC)

### 1b. Put cards in Pis

Once all 3 cards are flashed:

1. Put the `bene-gesserit` card into the Pi 4
2. Put the `morgan-le-fay` card into one Pi Zero 2W
3. Put the `kiki` card into the other Pi Zero 2W
4. Power them all on

## Step 2: First Boot

Each Pi needs about 60 seconds on first boot to:
- Resize the filesystem to fill the 128GB card
- Generate SSH keys
- Connect to WiFi

Then you can reach them:
```bash
ssh coven@bene-gesserit.local    # Pi 4
ssh coven@morgan.local        # Rover 1
ssh coven@kiki.local         # Rover 2
```

If `.local` doesn't resolve, you'll need to find their IP addresses
from your router's admin page or with `nmap -sn 192.168.x.0/24`.

## Step 3: Run Setup Scripts

From your tower, you're going to copy a setup script to each Pi and
run it. You can do all three in parallel — just open separate terminal
tabs.

### Dock (Pi 4):

1. Open a terminal on your tower
2. Copy the dock setup script to the Pi 4:
   ```bash
   cd ~/Desktop/Work/COVEN/Code/COVEN-CORE/setup
   scp setup_dock.sh coven@bene-gesserit.local:~
   ```
3. SSH into the Pi 4:
   ```bash
   ssh coven@bene-gesserit.local
   ```
4. Run the setup script:
   ```bash
   chmod +x setup_dock.sh
   ./setup_dock.sh
   ```
5. This installs Docker, serial tools, I2C tools, and udev rules.
   Takes about 10-15 minutes (apt + Docker install).
6. When it finishes, log out and back in so Docker permissions work:
   ```bash
   exit
   ssh coven@bene-gesserit.local
   ```

### Rovers (Pi Zero 2W):

Do this for each rover. Same steps, different hostnames.

1. Open a terminal on your tower
2. Copy the rover setup script to the Pi:
   ```bash
   cd ~/Desktop/Work/COVEN/Code/COVEN-CORE/setup
   scp setup_rover.sh coven@morgan.local:~    # or kiki.local
   ```
3. SSH into the rover:
   ```bash
   ssh coven@morgan.local    # or kiki.local
   ```
4. Run the setup script:
   ```bash
   chmod +x setup_rover.sh
   ./setup_rover.sh
   ```
5. This installs I2C tools, verifies USB gadget serial, creates the
   config directory, and sets up a systemd service (disabled until
   you deploy the firmware binary later).

   **This takes longer on the Pi Zero** — it's a single-core ARM chip
   doing apt updates. Go do something else for 20 minutes.

6. After setup finishes, edit the rover identity:
   ```bash
   sudo nano /etc/coven/rover.toml
   ```
   Change the line `rover_id = "CHANGE_ME"` to this rover's witch name:
   - On morgan: `rover_id = "Morgan_Le_Fay"`
   - On kiki: `rover_id = "Kiki"`

7. Repeat for the other rover.

## Step 4: Get Code onto the Dock

The dock Pi needs a copy of the COVEN-CORE codebase so it can run the
ROS2 nodes. You'll copy it from your tower to the Pi over the network.

1. Open a terminal on your tower
2. Sync the codebase to the dock:
   ```bash
   rsync -avz --exclude='.cache' --exclude='target' --exclude='.git' \
       ~/Desktop/Work/COVEN/Code/COVEN-CORE/ \
       coven@bene-gesserit.local:~/coven/COVEN-CORE/
   ```
   This copies everything except build artifacts. Use this same command
   any time you change code on your tower and want to update the dock.

3. SSH into the dock:
   ```bash
   ssh coven@bene-gesserit.local
   ```
4. Build the ROS2 Docker container (first time only, takes ~10 min):
   ```bash
   cd ~/coven/COVEN-CORE/coven_core/docker
   ./run.sh build
   ```
5. Launch the dock nodes:
   ```bash
   ./run.sh dock
   ```

## Step 5: Cross-Compile and Deploy Rover Firmware

The rovers run a Rust binary, not Python/ROS2. The Pi Zero 2W only has
512MB of RAM — not enough to compile Rust. So you compile the firmware
on your tower (which is fast) and send the finished binary to each Pi
over the network.

### One-time setup (on your tower):

1. Install the ARM64 cross-compilation target for Rust:
   ```bash
   rustup target add aarch64-unknown-linux-gnu
   ```
2. Install the ARM64 linker (so Rust can produce ARM binaries):
   ```bash
   sudo dnf install gcc-aarch64-linux-gnu
   ```

### Deploy to each rover:

1. From your tower, run the deploy script:
   ```bash
   cd ~/Desktop/Work/COVEN/Code/COVEN-CORE/setup
   ./deploy_rover.sh morgan    # Compiles and sends to rover 1
   ./deploy_rover.sh kiki     # Same binary, sends to rover 2
   ```
   The first compile takes a few minutes (downloading + building
   Rust crates). After that, incremental builds are fast.

2. SSH into each rover and start the service:
   ```bash
   ssh coven@morgan.local
   sudo systemctl enable --now coven-rover
   ```
   `enable` makes it start on boot. `--now` starts it immediately.

3. Watch the logs to see if it's working:
   ```bash
   sudo journalctl -u coven-rover -f
   ```
   Press Ctrl+C to stop watching.

## Step 6: Verify Everything

### Check the dock:
```bash
ssh coven@bene-gesserit.local
cd ~/coven/COVEN-CORE/coven_core/docker
./run.sh shell
# Inside the container:
ros2 topic list    # Should show /coven/* topics
```

### Check a rover:
```bash
ssh coven@morgan.local
sudo systemctl status coven-rover    # Should be "active (running)"
sudo journalctl -u coven-rover -n 20 # Recent log output
```

### Test dock-rover connection:
1. Plug rover into dock via USB cable
2. On the dock: `ls /dev/ttyACM*` — should show the rover's serial port
3. In the dock's ROS2 container, the rover_bridge node should detect it
   and start the handshake

## Quick Reference

| Task | Command |
|------|---------|
| Flash a card | `./flash_sd.sh <dock\|rover> <hostname>` |
| Set up the dock | `./setup_dock.sh` (run ON the Pi 4) |
| Set up a rover | `./setup_rover.sh` (run ON the Pi Zero) |
| Deploy firmware | `./deploy_rover.sh <hostname>` (run on dev machine) |
| Start dock | `cd docker && ./run.sh dock` |
| Start rover | `sudo systemctl enable --now coven-rover` |
| Rover logs | `sudo journalctl -u coven-rover -f` |
| Sync code to dock | `rsync -avz ... coven@bene-gesserit.local:~/coven/COVEN-CORE/` |
| Check I2C devices | `sudo i2cdetect -y 1` |
| Check serial ports | `ls /dev/ttyACM* /dev/ttyGS0 /dev/rplidar 2>/dev/null` |

## Troubleshooting

**Can't SSH to Pi:**
- Wait longer — first boot takes 60+ seconds
- Check that your dev machine is on the same WiFi network
- Try `ping morgan.local` — if it doesn't resolve, find the IP from your router

**Docker permission denied:**
- Log out and back in after running setup_dock.sh
- Or run `newgrp docker` in your current session

**No /dev/ttyACM0 when rover is plugged in:**
- Check that the rover's USB cable is data-capable (not charge-only)
- On the rover: `lsmod | grep g_serial` — should show the module loaded
- Check dmesg on the dock: `dmesg | tail -20`

**RPLIDAR not showing as /dev/rplidar:**
- Check USB connection to the LiDAR
- `dmesg | tail -10` to see if it was detected
- `ls /dev/ttyUSB*` — it might be there without the symlink
- Reload udev: `sudo udevadm control --reload-rules && sudo udevadm trigger`

**I2C device not found:**
- `sudo i2cdetect -y 1` — is 0x48 showing up?
- Check wiring: SDA to GPIO 2, SCL to GPIO 3, VCC to 3.3V, GND to GND
- Make sure `dtparam=i2c_arm=on` is in `/boot/firmware/config.txt`

**Rover firmware crashes on start:**
- Check the config: `cat /etc/coven/rover.toml`
- Run manually to see errors: `sudo /usr/local/bin/coven-rover --config /etc/coven/rover.toml`
- If it's a GPIO error, make sure you're running as root (the service does this)
