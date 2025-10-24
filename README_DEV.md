# 🧠 COVEN Developer Guide

This document covers local setup, development workflow, and structure of the **COVEN-CORE** repository — the ROS 2 foundation for the COVEN modular swarm system.

COVEN (Composable Operations for Versatile Exploration Networks) is a ROS 2–based modular robotics framework enabling autonomous dock-and-rover swarms with standardized mechanical, electrical, and data interfaces.  
Phase 1 focuses on simulation: validating lifecycle logic (*connect → verify → operate → return*) before hardware fabrication.

---

## 🧩 Repository Structure

coven_core/
├── coven_core/
│ ├── common.py # Shared dataclasses and enums (FSM messages)
│ ├── dock_node_multi.py # Multi-slot Dock manager
│ ├── module_node.py # Standard module implementation
│ ├── module_flaky.py # Flaky module variant (drops heartbeats)
│ ├── dock_node.py # (legacy) single-module Dock node
│ └── init.py
│
├── launch/ # Optional ROS 2 launch files
├── resource/ # Package manifest files
├── test/ # Unit / sim-test stubs
│
├── CMakeLists.txt
├── package.xml
├── pyproject.toml
├── setup.py
└── README.md


Supporting documentation:

docs/
├── COVEN_InterfaceSpec_v0.2.pdf
├── COVEN_Operator_Guide.md (alias: README.md)
├── README_DEV.md (this file)
└── CAD/ (connector, plug, and module models)


---

## ⚙️ Environment Setup

Tested on **Ubuntu 22.04 LTS** with **ROS 2 Humble**.

### 1. Install ROS 2 Humble
Follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Create workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-user>/coven-core.git
cd ..

3. Install dependencies

sudo apt update
rosdep install --from-paths src -yi

4. Build

colcon build --symlink-install
source install/setup.bash

🧪 Running in Simulation

Launch Gazebo (Ignition) TurtleBot world:

coven.sim

In separate terminals:

coven.dock     # start DockMulti
coven.module   # start a module (or coven.module &)
coven.mission  # send mission request

You can now observe:

    Dock ↔ Module handshake (IDENTIFY → VERIFY → ENABLE)

    Heartbeat monitoring and dropout recovery

    Mission dispatch and field-ops lifecycle

🧰 Development Workflow
Build Cycle

colcon build --symlink-install
source install/setup.bash

Code Style

    Python ≥ 3.10

    Follow PEP 8

Use Black

for formatting:

    black coven_core/

Logging Conventions

    Dock logs all system activity, heartbeats, and recoveries.

    Modules stay quiet except for init and task events.

    Colorized logs use ANSI sequences (green = OK, yellow/orange = warn, red = fault).

🔄 Git Workflow

Typical cycle:

git pull
# make edits
git add .
git commit -m "Refactor heartbeat timing and update InterfaceSpec link"
git push

Tag releases matching interface milestones:

git tag -a v0.2 -m "Phase 0 complete: interface + ROS2 FSM validated"
git push origin v0.2

🧠 Simulation Notes

    COVEN currently uses TurtleBot4 Gazebo (Ignition/Garden) for locomotion.

    LIDAR and wheel control topics:

        /scan → LaserScan

        /cmd_vel → Twist

        /odom → Odometry

    You can connect these directly to the Module node for autonomous behavior.

🔧 Extending the Framework

    Add new modules:
    Derive from coven_core.module_node.Module or module_flaky.FlakyModule.

    Add new Dock logic:
    Modify or extend dock_node_multi.py to handle new mission types.

    Hardware interface:
    The 9-pin Type-A plug (5-over-4 layout) and lifecycle logic are detailed in
    docs/COVEN_InterfaceSpec_v0.2.pdf.

📚 Phase References

    Interface Spec v0.2 — finalized pin layout, handshake FSM, and symbolic lifecycle

    Thesis Skeleton (2025) — structure for Phase 1 write-up

    CAD Envelope — 100 × 100 × 50 mm CubeRover plug housing

🧾 License

MIT-style license unless otherwise noted in submodules.

Built and maintained by Ander Shultis — 2025
University of Hawai‘i at Mānoa / Colorado School of Mines collaboration
Phase 1 of the COVEN modular swarm project.


---