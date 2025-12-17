#!/usr/bin/env python3
"""
COVEN Data Mule Simulation Launch

All simulation output goes to separate gnome-terminal windows:
1. COVEN - Infrastructure (Gazebo, bridges, SLAM)
2. {Coven Name} - Coven readouts (dispatcher, offline SLAM processor)
3. {Witch Name} - Witch readouts (one per witch)

Uses ROS2 event handlers for proper sequencing:
- Gazebo starts first
- Models spawn after Gazebo is ready (via OnProcessStart)
- Bridges start after models are spawned
- Witch/Dock nodes start after bridges are ready

Usage:
    ros2 launch coven_core coven_data_mule_sim.launch.py witch_count:=1
    ros2 launch coven_core coven_data_mule_sim.launch.py witch_count:=2
    ros2 launch coven_core coven_data_mule_sim.launch.py witch_count:=4

Author: Alexander Shultis
Date: December 2025
"""

import os
import random
import math
import shlex
from ament_index_python.packages import get_package_share_directory


def write_spawn_script(sdf_path: str, model_name: str, x: float, y: float, z: float,
                       qx: float = 0, qy: float = 0, qz: float = 0, qw: float = 1) -> str:
    """Write a bash script that spawns a model using gz service with inline SDF."""
    script_path = f'/tmp/spawn_{model_name}.sh'

    # Read SDF and prepare for protobuf text format
    with open(sdf_path, 'r') as f:
        sdf_content = f.read()

    # Remove XML declaration and collapse to single line
    sdf_content = sdf_content.replace('<?xml version="1.0"?>', '').strip()
    # Remove newlines and collapse whitespace for cleaner inline SDF
    sdf_content = ' '.join(sdf_content.split())
    # Escape double quotes for protobuf text format (backslash-escaped inside the outer quotes)
    sdf_escaped = sdf_content.replace('"', '\\"')

    # Build the request string with proper quoting
    # Use single quotes for outer shell, protobuf expects: sdf: "...", name: "..."
    req_str = f'sdf: "{sdf_escaped}", name: "{model_name}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}'

    script_content = f'''#!/bin/bash
source /opt/ros/jazzy/setup.bash

gz service -s /world/coven_world/create \\
    --reqtype gz.msgs.EntityFactory \\
    --reptype gz.msgs.Boolean \\
    --timeout 10000 \\
    --req '{req_str}'
'''
    with open(script_path, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    return script_path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    LogInfo,
    RegisterEventHandler,
    GroupAction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration

# Gazebo service/type strings
GZ_CREATE_SERVICE = '/world/coven_world/create'
GZ_ENTITY_FACTORY_TYPE = 'gz.msgs.EntityFactory'
GZ_BOOLEAN_TYPE = 'gz.msgs.Boolean'

# Witch names for rovers
WITCH_NAMES = [
    "Morgan_Le_Fay", "Hecate", "Circe", "Scathach", "Morrigan",
    "Lorelei", "Frau_Holle", "Louhi", "Baba_Yaga", "Mami_Wata",
    "Princess_Kaguya", "Elphaba", "Glinda", "Wanda_Maximoff",
    "Agatha_Harkness", "Zatanna_Zatara", "Hermione_Granger",
    "Minerva_McGonagall", "Sabrina_Spellman", "Willow_Rosenberg",
    "Maleficent", "Endora", "Samantha_Stephens", "Kiki", "Yubaba",
    "Akko", "Nico_Wakatsuki", "Mother_Talzin", "Old_Daka", "Axkva_Min",
]

# Coven names for docks
COVEN_NAMES = [
    "The_Graeae", "The_Erinyes", "The_Norns", "The_Weird_Sisters",
    "The_Sanderson_Sisters", "The_Hex_Girls", "The_Crones",
    "The_Hags_of_Dun_Broch", "The_Bene_Gesserit", "The_Lilim",
]


def write_coven_display_script(dock_name: str, dock_display: str, witch_list: list,
                                witch_displays: list, witch_count: int) -> str:
    """Write a simple display script for the COVEN terminal header."""
    script_path = '/tmp/coven_display.sh'

    witch_header_lines = ""
    for wd in witch_displays:
        witch_header_lines += f'echo "║  Witch: {wd:<52} ║"\n'

    script_content = f'''#!/bin/bash
# COVEN Infrastructure Display
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              COVEN {witch_count}-Witch Simulation                        ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Coven: {dock_display:<52} ║"
{witch_header_lines}echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Infrastructure starting..."
echo ""
# Keep terminal open - the actual processes run separately
tail -f /dev/null
'''
    with open(script_path, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    return script_path


def write_witch_script(witch_name: str, witch_display: str, witch_num: int,
                       witch_count: int, workspace: str, session_timestamp: str,
                       coven_name: str, spawn_x: float, spawn_y: float, spawn_yaw: float) -> str:
    """Write the bash script for a witch terminal."""
    script_path = f'/tmp/coven_witch_{witch_name}.sh'
    script_content = f'''#!/bin/bash
# Auto-generated launch script for witch {witch_name}
set -e

source /opt/ros/jazzy/setup.bash
source {workspace}/install/setup.bash 2>/dev/null || true

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                   {witch_display:^20} ║"
echo "║                        Witch {witch_num} of {witch_count}                           ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Role: Go where told, record sensors, return with data       ║"
echo "║  Reactive navigation - simple but reliable                   ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

echo "[{witch_name}] Waiting for simulation infrastructure..."

# Wait for scan and odom topics to be available
while ! ros2 topic list 2>/dev/null | grep -q "/{witch_name}/scan"; do
    sleep 0.5
done
while ! ros2 topic list 2>/dev/null | grep -q "/{witch_name}/odom"; do
    sleep 0.5
done

echo "[{witch_name}] Starting..."
echo ""

ros2 run coven_core data_mule_module --ros-args \\
    -p use_sim_time:=true \\
    -p module_id:={witch_name} \\
    -p robot_namespace:={witch_name} \\
    -p session_timestamp:={session_timestamp} \\
    -p coven_name:={coven_name} \\
    -p spawn_x:={spawn_x} \\
    -p spawn_y:={spawn_y} \\
    -p spawn_yaw:={spawn_yaw} \\
    -p record_rate_hz:=10.0 \\
    -p linear_speed:=0.4 \\
    -p angular_speed:=0.6 \\
    -p obstacle_threshold:=0.4
'''
    with open(script_path, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    return script_path


def write_dock_script(dock_name: str, dock_display: str, witch_displays: list,
                      witch_count: int, workspace: str, dock_x: float, dock_y: float,
                      session_timestamp: str) -> str:
    """Write the bash script for the coven (dock) terminal."""
    script_path = f'/tmp/coven_dock_{dock_name}.sh'

    # Data directory: ~/Desktop/COVEN/Data/YYYYMMDD.HHMM.SS/Coven_Name/
    data_dir = f'$HOME/Desktop/COVEN/Data/{session_timestamp}/{dock_name}'
    slam_dir = f'{data_dir}/SLAM'

    script_content = f'''#!/bin/bash
# Auto-generated launch script for coven {dock_name}
set -e

source /opt/ros/jazzy/setup.bash
source {workspace}/install/setup.bash 2>/dev/null || true

# Session data directory
DATA_DIR="{data_dir}"
SLAM_DIR="{slam_dir}"
mkdir -p "$DATA_DIR"
mkdir -p "$SLAM_DIR"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                  {dock_display:^20} ║"
echo "║                            Coven                             ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Role: Analyze maps, dispatch rovers to frontiers            ║"
echo "║  Offline SLAM: Process recorded sensor data                  ║"
echo "║  Frontier Dispatch: Direct exploration                       ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Session: {session_timestamp}                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Data directory: $DATA_DIR"
echo "SLAM output:    $SLAM_DIR"
echo ""

echo "[{dock_name}] Waiting for simulation infrastructure..."

# Wait for clock topic
while ! ros2 topic list 2>/dev/null | grep -q "^/clock$"; do
    sleep 0.5
done

# Start offline SLAM processor
echo "[{dock_name}] Starting Offline SLAM Processor..."
ros2 run coven_core offline_slam_processor --ros-args \\
    -p use_sim_time:=true \\
    -p playback_speed:=10.0 \\
    -p data_dir:="$DATA_DIR" \\
    -p slam_output_dir:="$SLAM_DIR" \\
    -p scan_topic:=/offline_scan \\
    -p odom_topic:=/offline_odom &
SLAM_PID=$!
sleep 2

# Start frontier dispatcher
echo "[{dock_name}] Starting Frontier Dispatcher..."
ros2 run coven_core frontier_dispatcher --ros-args \\
    -p use_sim_time:=true \\
    -p dock_position_x:={dock_x} \\
    -p dock_position_y:={dock_y} \\
    -p exploration_radius:=4.0 \\
    -p min_frontier_size:=3 \\
    -p coverage_goal:=0.75 \\
    -p auto_dispatch:=true &
DISPATCH_PID=$!

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                    COVEN STATUS MONITOR                      ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Function to save map
save_map() {{
    local map_name=$1
    echo "[{dock_name}] Saving map as $map_name..."
    ros2 run nav2_map_server map_saver_cli -f "$SLAM_DIR/$map_name" --ros-args -p use_sim_time:=true 2>/dev/null || \\
        echo "[{dock_name}] Map save failed (map may not be ready yet)"
}}

# Cleanup function - save map before exit
cleanup() {{
    echo ""
    echo "[{dock_name}] Shutting down..."
    echo "[{dock_name}] Saving map..."
    save_map "map"
    kill $SLAM_PID $DISPATCH_PID 2>/dev/null || true
    wait $SLAM_PID $DISPATCH_PID 2>/dev/null || true
    exit 0
}}
trap cleanup SIGINT SIGTERM EXIT

# Monitor dispatcher status - exit if background processes die
while kill -0 $DISPATCH_PID 2>/dev/null; do
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  $(date '+%H:%M:%S') - Dispatcher Status"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    ros2 topic echo /coven/dispatcher_status --once --no-arr 2>/dev/null | head -20 || echo "  Waiting for dispatcher..."
    sleep 5
done

echo "[{dock_name}] Dispatcher process ended, exiting..."
'''
    with open(script_path, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    return script_path


def launch_setup(context, *args, **kwargs):
    """Generate launch actions based on witch_count parameter."""

    witch_count = int(LaunchConfiguration('witch_count').perform(context))

    # Select random names
    shuffled_witches = WITCH_NAMES.copy()
    random.shuffle(shuffled_witches)
    witch_list = shuffled_witches[:witch_count]
    dock_name = random.choice(COVEN_NAMES)

    witch_displays = [name.replace('_', ' ') for name in witch_list]
    dock_display = dock_name.replace('_', ' ')

    # Package paths
    pkg_coven = get_package_share_directory('coven_core')
    models_path = os.path.join(pkg_coven, 'models')
    worlds_path = os.path.join(pkg_coven, 'worlds')
    workspace = os.environ.get('COLCON_PREFIX_PATH', '/home/avactus/ros2_ws/install').replace('/install', '')

    # Files
    slam_params_file = os.path.join(pkg_coven, 'config', 'slam_params_sim.yaml')
    rover_sdf_path = os.path.join(models_path, 'coven_rover', 'model.sdf')
    dock_sdf_path = os.path.join(models_path, 'dock.sdf')
    world_sdf = os.path.join(worlds_path, 'coven_4rover.sdf')

    dock_x, dock_y = 0.0, 0.0

    # Generate session timestamp for data organization
    # Format: YYYYMMDD.HHMM.SS (shared by all witches in this sim)
    from datetime import datetime
    session_timestamp = datetime.now().strftime('%Y%m%d.%H%M.%S')

    # Calculate rover positions
    rover_data = []
    for i in range(witch_count):
        angle = (2 * math.pi * i) / max(witch_count, 1)
        rx = dock_x + 1.2 * math.cos(angle)
        ry = dock_y + 1.2 * math.sin(angle)
        qz = math.sin(angle / 2)
        qw = math.cos(angle / 2)

        rover_data.append({
            'name': witch_list[i],
            'display': witch_displays[i],
            'x': rx, 'y': ry, 'qz': qz, 'qw': qw,
        })

    # Write scripts
    dock_script = write_dock_script(
        dock_name, dock_display, witch_displays, witch_count, workspace, dock_x, dock_y, session_timestamp
    )

    witch_scripts = []
    for i, rd in enumerate(rover_data):
        # Calculate yaw from quaternion (only z rotation, so yaw = 2 * atan2(qz, qw))
        spawn_yaw = 2.0 * math.atan2(rd['qz'], rd['qw'])
        script = write_witch_script(
            rd['name'], rd['display'], i + 1, witch_count, workspace, session_timestamp, dock_name,
            rd['x'], rd['y'], spawn_yaw
        )
        witch_scripts.append({'name': rd['name'], 'script': script})

    actions = []

    # Environment
    actions.append(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{worlds_path}'
    ))
    actions.append(SetEnvironmentVariable(
        name='QT_QPA_PLATFORM',
        value='xcb'
    ))

    # ============================================================
    # STAGE 1: Start Gazebo (direct ExecuteProcess for event handling)
    # ============================================================
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', world_sdf],
        output='screen',
        name='gazebo',
        # Set env directly on this process too
        additional_env={'GZ_SIM_RESOURCE_PATH': f'{models_path}:{worlds_path}'}
    )
    actions.append(gazebo_process)

    # ============================================================
    # STAGE 2: Clock bridge (starts 3s after Gazebo)
    # ============================================================
    clock_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        name='clock_bridge'
    )
    actions.append(TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[COVEN] Starting clock bridge...'),
            clock_bridge
        ]
    ))

    # ============================================================
    # STAGE 3: Spawn models (5s after Gazebo - give world time to load)
    # Using spawn scripts with inline SDF because sdf_filename doesn't render in Gazebo GUI
    # ============================================================
    spawn_actions = [LogInfo(msg='[COVEN] Spawning models...')]

    # Create spawn script for dock
    dock_spawn_script = write_spawn_script(dock_sdf_path, dock_name, dock_x, dock_y, 0.05)
    spawn_actions.append(ExecuteProcess(
        cmd=['bash', dock_spawn_script],
        output='screen',
        name='spawn_dock'
    ))

    # Create spawn scripts for rovers
    for rd in rover_data:
        rover_spawn_script = write_spawn_script(
            rover_sdf_path, rd['name'], rd['x'], rd['y'], 0.15,
            qz=rd['qz'], qw=rd['qw']
        )
        spawn_actions.append(ExecuteProcess(
            cmd=['bash', rover_spawn_script],
            output='screen',
            name=f'spawn_{rd["name"]}'
        ))

    actions.append(TimerAction(
        period=5.0,
        actions=spawn_actions
    ))

    # ============================================================
    # STAGE 4: Sensor bridges (8s after Gazebo - after models spawned)
    # ============================================================
    bridge_actions = [LogInfo(msg='[COVEN] Starting sensor bridges...')]

    for rd in rover_data:
        name = rd['name']
        # cmd_vel bridge (ROS -> Gazebo)
        bridge_actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '--ros-args', '-r', f'/model/{name}/cmd_vel:=/{name}/cmd_vel'
            ],
            output='screen',
            name=f'{name}_cmd_vel_bridge'
        ))
        # scan bridge (Gazebo -> ROS)
        bridge_actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/coven_world/model/{name}/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '--ros-args', '-r', f'/world/coven_world/model/{name}/link/base_link/sensor/lidar/scan:=/{name}/scan'
            ],
            output='screen',
            name=f'{name}_scan_bridge'
        ))
        # odom bridge (Gazebo -> ROS)
        bridge_actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '--ros-args', '-r', f'/model/{name}/odometry:=/{name}/odom'
            ],
            output='screen',
            name=f'{name}_odom_bridge'
        ))

    actions.append(TimerAction(
        period=8.0,
        actions=bridge_actions
    ))

    # ============================================================
    # STAGE 5: SLAM Toolbox (10s after Gazebo)
    # ============================================================
    actions.append(TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[COVEN] Starting SLAM Toolbox...'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                    f'slam_params_file:={slam_params_file}',
                    'use_sim_time:=true',
                    'scan_topic:=/offline_scan'
                ],
                output='screen',
                name='slam_toolbox'
            )
        ]
    ))

    # ============================================================
    # STAGE 6: Witch terminals (15s+ after Gazebo - bridges should be up)
    # ============================================================
    for i, ws in enumerate(witch_scripts):
        delay = 15.0 + (i * 2.0)
        witch_title = ws['name'].replace('_', ' ')
        actions.append(TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    # --wait makes terminal close when script exits
                    cmd=['gnome-terminal', '--wait', '--title', witch_title, '--', 'bash', ws['script']],
                    output='log',
                    name=f'{ws["name"]}_terminal'
                ),
            ]
        ))

    # ============================================================
    # STAGE 7: Dock terminal (after witches)
    # ============================================================
    dock_delay = 17.0 + (witch_count * 2.0)
    dock_title = dock_name.replace('_', ' ')
    actions.append(TimerAction(
        period=dock_delay,
        actions=[
            ExecuteProcess(
                # --wait makes terminal close when script exits
                cmd=['gnome-terminal', '--wait', '--title', dock_title, '--', 'bash', dock_script],
                output='log',
                name=f'{dock_name}_terminal'
            ),
        ]
    ))

    # Final info
    actions.append(TimerAction(
        period=dock_delay + 2.0,
        actions=[
            LogInfo(msg=f'[COVEN] Infrastructure ready. {witch_count} witch(es) spawned.'),
        ]
    ))

    return actions


def generate_launch_description():
    """Generate the launch description with witch_count argument."""

    return LaunchDescription([
        DeclareLaunchArgument(
            'witch_count',
            default_value='1',
            description='Number of witches (rovers) to spawn (1-4)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
