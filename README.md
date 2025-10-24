# COVEN-CORE

**COVEN** (Composable Operations for Versatile Exploration Networks) is a ROS 2–based modular robotics framework enabling autonomous dock-and-rover swarms with standardized mechanical, electrical, and data interfaces.  
**Phase 1** validates full plug-level lifecycle logic — *connect, verify, operate, and return* — in simulation before fabrication.

---

# 🛰️ COVEN Command Suite

Local command-line utilities for running and testing the **COVEN** swarm system in ROS 2.

**COVEN** (Coupled Operations via Extendable Nodes) is a modular, hot-dockable rover swarm.  
These scripts streamline day-to-day development: launching the sim, running the Dock, spawning Modules, and issuing missions.

---

## ⚡ Quick Start

After building your workspace (`colcon build`) and sourcing it:

```bash
source ~/ros2_ws/install/setup.bash

Run the full sequence:

sim        # Launch the TurtleBot4 simulation
dock -m    # Start the DockMulti hub controller
module 1   # Bring up one or more rover modules
mission    # Send the default mission (disconnect_and_explore)

Or combine everything with:

assemble   # Launch Dock, Module, and Sim together

That’s it — you’ve got a live multi-agent COVEN swarm simulation.
📂 Directory Layout

Each command is a small shell script stored in your workspace and symlinked into ~/bin for global access.

~/ros2_ws/
 ├── assemble.sh      # launch everything together
 ├── dock.sh          # run the DockMulti hub
 ├── module.sh        # start a rover module
 ├── mission.sh       # send a one-time mission task
 └── sim.sh           # launch TurtleBot4 Gazebo simulation

~/bin/
 ├── coven.assemble → ~/ros2_ws/assemble.sh
 ├── coven.dock     → ~/ros2_ws/dock.sh
 ├── coven.module   → ~/ros2_ws/module.sh
 ├── coven.mission  → ~/ros2_ws/mission.sh
 └── coven.sim      → ~/ros2_ws/sim.sh

All scripts source the workspace before running:

source ~/ros2_ws/install/setup.bash

⚙️ Shell Aliases

Aliases live in ~/.bash_aliases (or .bashrc) (or .zshrc):

alias assemble='coven.assemble'
alias dock='coven.dock'
alias module='coven.module'
alias mission='coven.mission'
alias sim='coven.sim'

After adding them, reload your shell:

source ~/.bashrc

Now you can use short, natural commands from anywhere.
🚀 Command Reference
Command	Purpose	Example
assemble	Bring up the full ROS 2 stack (Dock + Module + Sim)	assemble
dock	Launch the DockMulti hub controller	dock -m
module	Start a COVEN module node	module 1
sim	Start the TurtleBot4 Gazebo simulation	sim
mission	Send a single task to the Dock	mission disconnect_and_explore
🧭 Mission Syntax

The mission command publishes to /coven/mission_req through ROS 2:

mission <task_name>

If no argument is given, it defaults to:

mission disconnect_and_explore

Example alternative tasks:

mission sample_and_return
mission recharge

🔧 Extending the Suite

To add new commands:

    Create a new script in ~/ros2_ws/

    Make it executable:

chmod +x ~/ros2_ws/<name>.sh

Symlink it into ~/bin:

    ln -s ~/ros2_ws/<name>.sh ~/bin/coven.<name>

    Add an alias to your shell config for short form.

This pattern keeps your workspace clean and gives you a portable operator interface that works anywhere on the system.
🧩 Example New Script

#!/bin/bash
# coven.monitor — display live heartbeat status
source ~/ros2_ws/install/setup.bash
ros2 topic echo /coven/heartbeat

📜 License

MIT-style, unless otherwise noted within submodules.

Built by Ander Shultis — 2025, Phase 1 development of the COVEN Swarm.


---