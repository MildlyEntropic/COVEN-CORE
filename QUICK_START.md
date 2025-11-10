# COVEN Quick Start

Get up and running with COVEN in minutes.

## Installation

### First Time
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Install global launcher
chmod +x coven
mkdir -p ~/.local/bin
cp coven ~/.local/bin/assemble
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Now `assemble` works from anywhere!

### Already Installed?
```bash
assemble build        # Rebuild if needed
assemble test         # Run tests
```

## Quick Test

```bash
# Interactive menu (easiest!)
assemble

# Or directly
assemble test         # 54 tests in 36 seconds
```

## Launch System

### Option 1: Interactive Menu
```bash
assemble              # Pick options from menu
```

### Option 2: Direct Commands

#### Terminal 1 - Dock
```bash
assemble dock
```

#### Terminal 2 - Module
```bash
assemble module       # Auto ID
# OR
assemble module RR-001   # Specific ID
```

#### Terminal 3 - Monitor
```bash
assemble monitor      # Pick topic to watch
```

## Common Tasks

### Run Tests
```bash
assemble test         # Comprehensive (36s, 54 tests)
assemble test-quick   # Quick sanity (3s, 38 tests)
```

### Build & Rebuild
```bash
assemble build        # Build workspace
assemble rebuild      # Clean + build
assemble clean        # Clean artifacts
```

### Debug
```bash
assemble logs         # View recent logs
assemble version      # System info
assemble monitor      # Watch topics
```

## Manual Testing

### Test Discovery Protocol
```bash
# Terminal 1 - Start dock
assemble dock

# Terminal 2 - Start module
assemble module TEST-001

# Watch the logs:
# [INFO] [dock]: Sending IDENTIFY_REQ...
# [INFO] [module]: IDENTIFY_REQ received → responding
# [INFO] [dock]: IDENTIFY_REP received from TEST-001
# [WARN] [module]: VERIFY_REQ → replying FAIL (no hardware)
# [INFO] [module]: Heartbeat started for TEST-001
```

### Test Heartbeats
```bash
# Terminal 1
assemble dock

# Terminal 2
assemble module

# Terminal 3 - Watch heartbeats
ros2 topic echo /coven/heartbeat

# You should see:
# module_id: "..."
# seq: 1
# timestamp_sec: ...
# status: "ready"
```

### Test Multi-Module
```bash
# Terminal 1
assemble dock

# Terminal 2, 3, 4 - Start 3 modules
assemble module RR-001
assemble module RR-002
assemble module RR-003

# Terminal 5 - Monitor
ros2 topic echo /coven/heartbeat

# Watch 3 different modules reporting!
```

## Development Workflow

```bash
# 1. Edit code in your IDE

# 2. Quick test
assemble test-quick    # 3 seconds

# 3. If passes, full test
assemble test          # 36 seconds

# 4. All green? Commit!
```

## Troubleshooting

### Command not found: assemble
```bash
# Reload shell
source ~/.bashrc

# Or use local version
cd ~/ros2_ws
./coven test
```

### Tests fail: Module import errors
```bash
assemble rebuild
```

### Tests fail: pytest not found
```bash
sudo apt install python3-pytest
```

### Health check failures
This is **expected** without hardware! The tests use `skip_health_check=true` automatically.

### Need more help?
See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) or [ASSEMBLE_GUIDE.md](../../ASSEMBLE_GUIDE.md)

## Next Steps

- **Run tests**: `assemble test` (36s, proves everything works)
- **Read docs**: [TEST_ME.md](../../TEST_ME.md) (detailed test guide)
- **Learn testing**: [RIGOROUS_TESTING.md](../../RIGOROUS_TESTING.md) (why tests are rigorous)
- **Full commands**: [ASSEMBLE_GUIDE.md](../../ASSEMBLE_GUIDE.md) (complete reference)

---

**Quick Start:** `assemble` (that's it!)
