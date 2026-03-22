# COVEN Lightweight Rover (Rust)

Lightweight rover controller for COVEN-compliant reconnaissance modules.
No ROS2 dependency - controls hardware directly via rppal.

## CRITICAL: Communication Architecture

> **⚠️ THERE IS NO WIRELESS COMMUNICATION ON COVEN ROVERS ⚠️**

COVEN rovers do **NOT** use:
- WiFi
- Ethernet
- RF/Radio
- Bluetooth
- Any wireless protocol

### How Rovers Communicate

Rovers communicate with the dock **ONLY** when physically connected via the
**COVEN Type-A 9-pin connector**. Communication uses **UART** over the
connector's data lines (pins 7/8).

When deployed on a mission, the rover operates **completely autonomously**
with **no communication link** to the dock.

### Data-Mule Architecture

This is the COVEN **data-mule architecture** as defined in the Interface
Specification v0.2:

1. **DOCKING**: Rover physically docks with the dock station
2. **HANDSHAKE**: Dock identifies and verifies rover via UART
3. **MISSION**: Dock assigns mission (waypoints) via UART
4. **UNDOCK**: Rover physically detaches from dock
5. **EXPLORE**: Rover executes mission **autonomously** (NO COMMUNICATION)
6. **RETURN**: Rover navigates back to dock
7. **DOCK**: Rover physically re-docks
8. **UPLOAD**: Rover uploads collected sensor data via UART
9. **RECHARGE**: Dock charges rover battery

### Protocol Reference

See: `COVEN Interface Specification v0.3`
- Document: `20250808.ShultisAnder.COVEN.CAD.InterfaceSpec.pdf`
- Frame format: `[LEN_HI] [LEN_LO] [COBS-encoded data] [0x00]`
- COBS data encodes: `[TYPE] [PAYLOAD] [CRC]`
- Physical layer: UART @ 115200 baud via 9-pin connector

## Building

```bash
# Cross-compile for Raspberry Pi Zero 2W (64-bit Pi OS, default)
cargo build --release --target aarch64-unknown-linux-gnu

# Cross-compile for 32-bit Pi OS
cargo build --release --target armv7-unknown-linux-gnueabihf
```

## Usage

```bash
# Run with real hardware (on Raspberry Pi)
./coven-rover

# Run hardware diagnostics
./coven-rover --diag

# Run with mock hardware (for testing logic)
./coven-rover --mock

# Use specific config file
./coven-rover -c /path/to/rover.toml

# Verbose logging
./coven-rover -v
```

## Configuration

Configuration is loaded from TOML files. Search order:
1. `/etc/coven/rover.toml`
2. `~/.config/coven/rover.toml`
3. `./rover.toml`

Example configuration:

```toml
rover_id = "Morgan_Le_Fay"
coven_name = "The_Graeae"

# UART communication with dock (NOT WiFi!)
[dock_uart]
port = "/dev/ttyAMA0"  # Pi GPIO UART connected to 9-pin dock connector
baud_rate = 115200
retry_delay_secs = 1.0
max_retries = 10

[hardware.motors]
left_pwm = 12
left_in1 = 5
left_in2 = 6
right_pwm = 13
right_in1 = 16
right_in2 = 26
standby = 17
pwm_frequency = 1000.0
wheel_base = 0.298
wheel_radius = 0.0325
max_rpm = 130.0

[hardware.encoders]
left_a = 23
left_b = 24
right_a = 27
right_b = 22
pulses_per_rev = 312

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
```

## Hardware Requirements

- Raspberry Pi Zero 2W (or Pi 4)
- TB6612FNG Motor Driver
- JGA25-371 motors with encoders
- RPLIDAR C1
- ADS1015/ADS1115 ADC for battery monitoring
- COVEN Type-A 9-pin dock connector

## Module Structure

- `main.rs` - Entry point, CLI parsing
- `config.rs` - Configuration structures (TOML)
- `dock_uart.rs` - UART communication with dock (9-pin connector)
- `state.rs` - Rover FSM (lifecycle management)
- `protocol.rs` - COVEN protocol messages and framing
- `navigation.rs` - Lyapunov potential field navigation
- `lidar.rs` - RPLIDAR C1 driver
- `hardware/` - Motor, encoder, battery drivers
- `mock.rs` - Simulated hardware for testing

## Author

Alexander Shultis
January 2026
