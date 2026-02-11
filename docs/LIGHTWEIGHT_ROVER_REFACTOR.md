# Lightweight Rover Refactor: Dropping ROS2 on Rovers

**Author:** Alexander Shultis
**Date:** January 11, 2026
**Branch:** `feature/lightweight-rover`
**Status:** PLANNING

---

## Motivation

Pi Zero 2W has 512MB RAM. ROS2 Humble with DDS middleware consumes significant resources before any application code runs. For the data-mule architecture, rovers perform minimal computation - they don't need the full ROS2 stack.

**Goal:** Run rovers without ROS2, keeping the COVEN protocol intact, while the Pi 4 dock continues to run ROS2 for SLAM/Nav2/coordination.

---

## Current Architecture (ROS2 Everywhere)

```
┌─────────────────────────────────────────────────────────────┐
│                      Pi 4 DOCK                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ ROS2 Nodes  │  │ SLAM        │  │ frontier_dispatcher │  │
│  │ (dock_node) │  │ Toolbox     │  │                     │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│                         │                                   │
│                    ROS2 DDS (WiFi)                          │
└─────────────────────────┬───────────────────────────────────┘
                          │
        ┌─────────────────┴─────────────────┐
        │                                   │
        ▼                                   ▼
┌───────────────────┐               ┌───────────────────┐
│  Pi Zero 2W       │               │  Pi Zero 2W       │
│  ROVER 1          │               │  ROVER 2          │
│  ┌─────────────┐  │               │  ┌─────────────┐  │
│  │ ROS2 Nodes  │  │               │  │ ROS2 Nodes  │  │
│  │ motor_driver│  │               │  │ motor_driver│  │
│  │ encoder_odom│  │               │  │ encoder_odom│  │
│  │ ydlidar     │  │               │  │ ydlidar     │  │
│  │ data_mule   │  │               │  │ data_mule   │  │
│  └─────────────┘  │               └─────────────────┘  │
└───────────────────┘               └───────────────────┘
```

**Problems:**
- ROS2 + DDS on 512MB RAM is tight
- Cross-compiling ROS2 for armhf is painful
- Boot time is slow
- WiFi DDS discovery is flaky

---

## Proposed Architecture (No ROS2 on Rovers)

```
┌─────────────────────────────────────────────────────────────┐
│                      Pi 4 DOCK                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ ROS2 Nodes  │  │ SLAM        │  │ frontier_dispatcher │  │
│  │ (dock_node) │  │ Toolbox     │  │                     │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│         │                                                   │
│  ┌──────┴──────┐                                            │
│  │ rover_bridge│  ◄── New node: translates TCP ↔ ROS2      │
│  └─────────────┘                                            │
│         │                                                   │
│    TCP Socket (WiFi)                                        │
└─────────────┬───────────────────────────────────────────────┘
              │
      ┌───────┴───────┐
      │               │
      ▼               ▼
┌───────────────┐  ┌───────────────┐
│ Pi Zero 2W    │  │ Pi Zero 2W    │
│ ROVER 1       │  │ ROVER 2       │
│ ┌───────────┐ │  │ ┌───────────┐ │
│ │ Python    │ │  │ │ Python    │ │
│ │ daemon    │ │  │ │ daemon    │ │
│ │ (no ROS2) │ │  │ │ (no ROS2) │ │
│ └───────────┘ │  │ └───────────┘ │
│  - pigpio     │  │  - pigpio     │
│  - ydlidar SDK│  │  - ydlidar SDK│
│  - TCP client │  │  - TCP client │
└───────────────┘  └───────────────┘
```

---

## What Changes

### 1. ROVER SIDE (New: Pure Python Daemon)

**File:** `lightweight_rover/rover_daemon.py`

Replace all ROS2 nodes with a single Python daemon:

```python
# No rclpy, no ROS2 dependencies

class RoverDaemon:
    """Lightweight rover controller - no ROS2."""

    def __init__(self, rover_id, dock_ip, dock_port):
        self.rover_id = rover_id
        self.sock = None  # TCP connection to dock

        # Hardware interfaces (direct, no ROS2)
        self.motors = MotorController()      # pigpio PWM
        self.encoders = EncoderReader()      # pigpio interrupts
        self.lidar = YDLidarDriver()         # ydlidar SDK

        # State
        self.state = "BOOT"
        self.recording = False
        self.mission_data = []

    def connect_to_dock(self):
        """Establish TCP connection to dock."""
        pass

    def send_message(self, msg_type, payload):
        """Send COVEN protocol message over TCP."""
        # Same string format as ROS2 version
        # "HEARTBEAT:rover_id:battery:status:x:y:theta"
        pass

    def handle_command(self, cmd):
        """Process command from dock."""
        # TASK_REQ, ENABLE_POWER, etc.
        pass

    def control_loop(self):
        """Main loop: read sensors, execute behaviors, send updates."""
        while True:
            # Read sensors
            scan = self.lidar.get_scan()
            odom = self.encoders.get_odometry()

            # Reactive behavior (obstacle avoidance)
            cmd_vel = self.compute_velocity(scan)
            self.motors.set_velocity(cmd_vel)

            # Record if in mission
            if self.recording:
                self.mission_data.append({
                    'timestamp': time.time(),
                    'scan': scan,
                    'odom': odom
                })

            # Send heartbeat
            self.send_heartbeat()

            time.sleep(0.05)  # 20Hz
```

**Dependencies (rover side):**
- Python 3.9+ (comes with Raspberry Pi OS)
- `pigpio` - hardware PWM and GPIO interrupts
- `ydlidar` - Python SDK for YDLiDAR X4
- `numpy` - optional, for scan processing
- Standard library: `socket`, `json`, `threading`, `time`

**No ROS2. No DDS. No colcon build.**

---

### 2. DOCK SIDE (New: rover_bridge Node)

**File:** `coven_core/rover_bridge.py`

New ROS2 node that bridges TCP ↔ ROS2:

```python
import rclpy
from rclpy.node import Node
import socket
import threading

class RoverBridge(Node):
    """Bridge between lightweight rovers (TCP) and ROS2."""

    def __init__(self):
        super().__init__('rover_bridge')

        # TCP server for rover connections
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('0.0.0.0', 5555))
        self.server.listen(10)

        self.rovers = {}  # rover_id -> socket

        # ROS2 publishers (same topics as before)
        self.scan_pubs = {}      # /rover_id/scan
        self.odom_pubs = {}      # /rover_id/odom
        self.heartbeat_pub = self.create_publisher(String, '/coven/dock/heartbeat', 10)

        # ROS2 subscribers (commands to rovers)
        self.cmd_vel_subs = {}   # /rover_id/cmd_vel
        self.task_req_sub = ...  # /coven/{rover_id}/task_req

        # Accept connections in background
        threading.Thread(target=self.accept_connections, daemon=True).start()

    def accept_connections(self):
        """Accept incoming rover TCP connections."""
        while True:
            conn, addr = self.server.accept()
            # Handshake to get rover_id
            # Create publishers/subscribers for this rover
            # Start receiver thread

    def handle_rover_message(self, rover_id, msg):
        """Convert TCP message to ROS2."""
        if msg.startswith('SCAN:'):
            # Parse and publish LaserScan
            pass
        elif msg.startswith('ODOM:'):
            # Parse and publish Odometry
            pass
        elif msg.startswith('HEARTBEAT:'):
            # Forward to /coven/dock/heartbeat
            pass
        elif msg.startswith('MISSION_DATA:'):
            # Save to file for offline_slam_processor
            pass

    def forward_cmd_vel(self, rover_id, twist_msg):
        """Convert ROS2 Twist to TCP command."""
        cmd = f"CMD_VEL:{twist_msg.linear.x}:{twist_msg.angular.z}"
        self.rovers[rover_id].send(cmd.encode())
```

**The dock doesn't change much** - existing nodes (dock_node, frontier_dispatcher, offline_slam_processor) keep working. They just talk to rover_bridge instead of directly to rover ROS2 nodes.

---

### 3. PROTOCOL CHANGES

The COVEN protocol messages stay the same format - they're just strings. Transport changes from ROS2 topics to TCP:

| Message | ROS2 (Current) | TCP (New) |
|---------|----------------|-----------|
| IDENTIFY_REQ | `/coven/dock/identify_req` | TCP broadcast to all rovers |
| IDENTIFY_REP | `/coven/dock/identify_rep` | TCP response |
| HEARTBEAT | `/coven/dock/heartbeat` | TCP, 1Hz |
| TASK_REQ | `/coven/{id}/task_req` | TCP to specific rover |
| LaserScan | `/{id}/scan` | TCP binary/JSON, bridge publishes to ROS2 |
| Odometry | `/{id}/odom` | TCP binary/JSON, bridge publishes to ROS2 |
| cmd_vel | `/{id}/cmd_vel` | TCP from bridge to rover |

**Wire format options:**
1. **JSON** - human readable, easy debug, larger
2. **MessagePack** - binary, compact, fast
3. **Custom binary** - most efficient, most work

Recommend: JSON for prototype, MessagePack if bandwidth becomes issue.

---

### 4. HARDWARE DRIVERS TO PORT

#### Motor Driver (pigpio version)

Current: `motor_driver.py` (ROS2 node)
New: `lightweight_rover/motors.py` (pure Python class)

```python
import pigpio

class MotorController:
    """TB6612FNG motor control via pigpio."""

    # Pin definitions (same as ROS2 version)
    LEFT_PWM = 12
    LEFT_IN1 = 5
    LEFT_IN2 = 6
    RIGHT_PWM = 13
    RIGHT_IN1 = 16
    RIGHT_IN2 = 26
    STBY = 17

    def __init__(self):
        self.pi = pigpio.pi()
        # Setup pins...

    def set_velocity(self, linear, angular):
        """Differential drive kinematics."""
        # Same math as ROS2 version
        pass

    def stop(self):
        """Emergency stop."""
        pass
```

#### Encoder Reader (pigpio version)

Current: `encoder_odom.py` (ROS2 node)
New: `lightweight_rover/encoders.py` (pure Python class)

```python
import pigpio

class EncoderReader:
    """Quadrature encoder reading via pigpio callbacks."""

    LEFT_A, LEFT_B = 23, 24
    RIGHT_A, RIGHT_B = 27, 22

    def __init__(self, ppr=210, wheel_radius=0.1, wheel_base=0.298):
        self.pi = pigpio.pi()
        self.left_ticks = 0
        self.right_ticks = 0
        # Setup callbacks...

    def get_odometry(self):
        """Return current x, y, theta, vx, vtheta."""
        # Same math as ROS2 version
        pass
```

#### YDLiDAR Driver

Current: `ydlidar_ros2_driver` (external ROS2 package)
New: Use YDLiDAR Python SDK directly

```python
from ydlidar import YDLidar

class LidarDriver:
    """YDLiDAR X4 driver using Python SDK."""

    def __init__(self, port='/dev/ydlidar'):
        self.lidar = YDLidar()
        self.lidar.connect(port, 128000)
        self.lidar.start_scan()

    def get_scan(self):
        """Return list of (angle, distance) tuples."""
        return self.lidar.get_scan_data()
```

---

### 5. FILE STRUCTURE

```
coven_core/
├── coven_core/              # Existing ROS2 code (dock side)
│   ├── dock_node.py
│   ├── frontier_dispatcher.py
│   ├── offline_slam_processor.py
│   └── rover_bridge.py      # NEW: TCP ↔ ROS2 bridge
│
├── lightweight_rover/       # NEW: No-ROS2 rover code
│   ├── __init__.py
│   ├── rover_daemon.py      # Main entry point
│   ├── motors.py            # TB6612FNG driver
│   ├── encoders.py          # Quadrature encoder driver
│   ├── lidar.py             # YDLiDAR driver
│   ├── protocol.py          # COVEN message parsing
│   ├── tcp_client.py        # Connection to dock
│   ├── behaviors.py         # Reactive navigation
│   └── config.py            # Hardware pin definitions
│
├── launch/
│   ├── coven_dock_hardware.launch.py    # Add rover_bridge
│   └── (rover launch files deprecated)
│
└── scripts/
    └── install_rover.sh     # Setup script for Pi Zero 2W
```

---

### 6. INSTALLATION (Rover Side)

**No ROS2 build required.** Just:

```bash
#!/bin/bash
# install_rover.sh - Run on Pi Zero 2W

# System packages
sudo apt update
sudo apt install -y python3-pip pigpio python3-pigpio

# Start pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# YDLiDAR udev rule
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar.rules
sudo udevadm control --reload-rules

# Python packages
pip3 install numpy msgpack

# YDLiDAR Python SDK
pip3 install ydlidar

# Copy lightweight_rover code
# (scp from dev machine or git clone)

# Create systemd service
sudo tee /etc/systemd/system/coven-rover.service << EOF
[Unit]
Description=COVEN Rover Daemon
After=network.target pigpiod.service

[Service]
ExecStart=/usr/bin/python3 /home/pi/coven/rover_daemon.py
WorkingDirectory=/home/pi/coven
User=pi
Restart=always

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable coven-rover
sudo systemctl start coven-rover
```

**Total install time: ~5 minutes** (vs hours for ROS2)

---

### 7. MIGRATION PATH

#### Phase 1: Create lightweight_rover package
- [ ] Port motor_driver.py → motors.py (remove rclpy)
- [ ] Port encoder_odom.py → encoders.py (remove rclpy)
- [ ] Create lidar.py wrapper around ydlidar SDK
- [ ] Create protocol.py for COVEN message parsing
- [ ] Create tcp_client.py for dock connection
- [ ] Create rover_daemon.py main loop

#### Phase 2: Create rover_bridge on dock
- [ ] Create rover_bridge.py ROS2 node
- [ ] TCP server accepting rover connections
- [ ] Translate TCP messages → ROS2 topics
- [ ] Translate ROS2 cmd_vel → TCP commands
- [ ] Handle mission data file transfer

#### Phase 3: Integration testing
- [ ] Test motors/encoders standalone on Pi Zero 2W
- [ ] Test LiDAR standalone
- [ ] Test TCP connection to dock
- [ ] Test full data-mule mission flow
- [ ] Verify SLAM works with bridged scan data

#### Phase 4: Cleanup
- [ ] Remove ROS2 rover launch files (or mark deprecated)
- [ ] Update documentation
- [ ] Create install_rover.sh script
- [ ] Test cold boot → mission complete flow

---

### 8. RISKS & MITIGATIONS

| Risk | Impact | Mitigation |
|------|--------|------------|
| TCP unreliable over WiFi | Lost messages, rover disconnect | Implement heartbeat timeout, auto-reconnect, message ACKs |
| Clock sync between rover/dock | SLAM timestamp errors | NTP sync, or dock timestamps all messages |
| YDLiDAR Python SDK issues | No scan data | Fall back to direct serial parsing |
| pigpio not available | No motor control | gpiod fallback (already in motor_driver.py) |
| Bandwidth for scan streaming | Lag, dropped scans | Compress scans, reduce rate, or don't stream (data-mule only) |

---

### 9. DECISION: Stream vs Store-and-Forward

**Option A: Stream scans over TCP**
- Pro: Real-time SLAM on dock
- Con: Bandwidth (~50KB/s per rover), latency sensitive

**Option B: Store on rover, transfer on dock (true data-mule)**
- Pro: No real-time bandwidth requirements
- Con: No live mapping until rover returns

**Recommendation:** Start with Option B (store-and-forward) since that's what data-mule architecture already does. Add streaming later if needed.

---

### 10. ESTIMATED EFFORT

| Task | Time |
|------|------|
| Port motor/encoder drivers | 2-3 hours |
| LiDAR wrapper | 1-2 hours |
| TCP client/protocol | 2-3 hours |
| rover_daemon main loop | 2-3 hours |
| rover_bridge ROS2 node | 3-4 hours |
| Integration testing | 4-6 hours |
| Install script & docs | 1-2 hours |
| **Total** | **~20 hours** |

---

## Summary

Dropping ROS2 on rovers means:
- **Faster boot** (seconds vs minutes)
- **Lower RAM usage** (~50MB vs ~300MB+)
- **Simpler deployment** (pip install vs colcon build)
- **Same COVEN protocol** (just different transport)
- **Dock unchanged** (ROS2 stays for SLAM/Nav2)

The tradeoff is maintaining two codebases (ROS2 dock + Python rover), but the rover code is small and focused.

---

*Document created: January 11, 2026*
*Branch: feature/lightweight-rover*
