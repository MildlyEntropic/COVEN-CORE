#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
coven_sim_proxy_1rover.launch.py — Single-rover Gazebo simulation with the
COVEN protocol proxy bridging the simulated rover to the live dock.

Architecture this brings up:

    Gazebo Harmonic (gz sim)
      └── coven_world with one coven_rover model named '<rover_name>'
            ├── /<rover_name>/scan       (LaserScan @ 10 Hz)
            ├── /<rover_name>/odometry   (Odometry @ 50 Hz)
            └── /<rover_name>/cmd_vel    (Twist input)
                              ↓
    ros_gz_bridge parameter_bridge
      Bridges the Gazebo topics above to ROS2 typed topics.
                              ↓
    sim_rover_proxy node ('coven_core sim_rover_proxy')
      └── Owns a PTY pair; speaks COVEN UART protocol on the master end.
          The slave path is logged at startup; the dock connects to it.
                              ↓
    rover_bridge node (unmodified production dock code)
      └── Auto-detects /dev/tty* devices, opens the proxy's PTY slave,
          handshakes via COVEN, dispatches tasks.

Usage (inside the COVEN sim Docker container):
    ros2 launch coven_core coven_sim_proxy_1rover.launch.py

Optional arguments:
    rover_name:=witch_morgan       Gazebo model + COVEN module_id
    capabilities:=0x03             Capability bitmask declared to dock
    module_type:=ReconRover        Type byte tag in IDENTIFY_REPLY
    world_file:=<path>             Override the default sim world
    headless:=true                 Run gz sim with -s (no GUI)
    pty_symlink:=/tmp/coven_sim_uart  Stable path the dock can target

Author: Alexander Shultis
Date: April 2026
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Gazebo service constants (Harmonic)
GZ_CREATE_SERVICE = '/world/coven_world/create'
GZ_ENTITY_FACTORY_TYPE = 'gz.msgs.EntityFactory'
GZ_BOOLEAN_TYPE = 'gz.msgs.Boolean'


def generate_launch_description():
    pkg_share = get_package_share_directory('coven_core')
    sim_dir = os.path.join(pkg_share, 'sim')
    default_world = os.path.join(sim_dir, 'worlds', 'coven_test.sdf')
    rover_sdf = os.path.join(sim_dir, 'models', 'coven_rover', 'model.sdf')
    models_path = os.path.join(sim_dir, 'models')

    rover_name_arg = DeclareLaunchArgument(
        'rover_name', default_value='witch_morgan',
        description='Gazebo model name and COVEN module_id for this rover.',
    )
    capabilities_arg = DeclareLaunchArgument(
        'capabilities', default_value='0x03',
        description='Capability bitmask the rover declares in IDENTIFY_REPLY.',
    )
    module_type_arg = DeclareLaunchArgument(
        'module_type', default_value='ReconRover',
        description='Module type label (and type byte tag) in IDENTIFY_REPLY.',
    )
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value=default_world,
        description='Path to the Gazebo SDF world file.',
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run gz sim with -s (no GUI) when true.',
    )
    pty_symlink_arg = DeclareLaunchArgument(
        'pty_symlink', default_value='/tmp/coven_sim_uart',
        description='Stable filesystem path the dock can target as the rover '
                    'serial device. The proxy creates this symlink to the '
                    'PTY slave path it owns.',
    )

    rover_name = LaunchConfiguration('rover_name')
    capabilities = LaunchConfiguration('capabilities')
    module_type = LaunchConfiguration('module_type')
    world_file = LaunchConfiguration('world_file')
    headless = LaunchConfiguration('headless')
    pty_symlink = LaunchConfiguration('pty_symlink')

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', models_path,
    )

    # ------------------------------------------------------------------
    # Stage 1: Gazebo simulation
    # ------------------------------------------------------------------
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', world_file],
        output='screen',
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '1', world_file],
        output='screen',
        condition=IfCondition(headless),
    )

    # ------------------------------------------------------------------
    # Stage 2: Clock bridge (Gazebo /clock → ROS2 /clock)
    # ------------------------------------------------------------------
    clock_bridge = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg='[sim] Starting /clock bridge'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                output='screen',
            ),
        ],
    )

    # ------------------------------------------------------------------
    # Stage 3: Spawn the rover model in Gazebo
    # ------------------------------------------------------------------
    spawn_rover = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg='[sim] Spawning rover'),
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', GZ_CREATE_SERVICE,
                    '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                    '--reptype', GZ_BOOLEAN_TYPE,
                    '--timeout', '5000',
                    '--req',
                    [
                        f'sdf_filename: "{rover_sdf}", name: "',
                        rover_name,
                        '", pose: {position: {x: 0.0, y: 0.0, z: 0.15}}',
                    ],
                ],
                output='screen',
            ),
        ],
    )

    # ------------------------------------------------------------------
    # Stage 4: Per-rover topic bridges
    # ------------------------------------------------------------------
    # Bridge Gazebo's per-model topics to ROS2 topics under /<rover_name>/.
    # Format: <ros_topic>@<ros_type>[<gz_type>  (or ] for input-only on ROS side)
    rover_topic_bridge = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[sim] Starting rover topic bridges'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    # Scan: Gazebo → ROS2
                    [
                        '/', rover_name,
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    ],
                    # Odometry: Gazebo → ROS2
                    [
                        '/model/', rover_name,
                        '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    ],
                    # cmd_vel: ROS2 → Gazebo
                    [
                        '/model/', rover_name,
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    ],
                ],
                output='screen',
            ),
        ],
    )

    # ------------------------------------------------------------------
    # Stage 5: The COVEN protocol proxy
    # ------------------------------------------------------------------
    sim_proxy = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[sim] Starting sim_rover_proxy'),
            Node(
                package='coven_core',
                executable='sim_rover_proxy',
                name='sim_rover_proxy',
                output='screen',
                parameters=[{
                    'rover_name': rover_name,
                    'module_type': module_type,
                    'capabilities': capabilities,
                    'gz_topic_ns': rover_name,
                    'pty_symlink': pty_symlink,
                    'use_sim_time': True,
                }],
            ),
        ],
    )

    # ------------------------------------------------------------------
    # Stage 6: The dock-side rover_bridge (unmodified production code)
    #
    # The rover_bridge auto-detects /dev/tty* candidates. The proxy creates
    # a symlink at <pty_symlink> pointing at its PTY slave; the dock can
    # be pointed there directly via the serial_port parameter, OR the
    # symlink can be created at /dev/ttyACMxx for auto-detect to find it.
    # ------------------------------------------------------------------
    dock_bridge = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[sim] Starting dock rover_bridge'),
            Node(
                package='coven_core',
                executable='rover_bridge',
                name='rover_bridge',
                output='screen',
                parameters=[{
                    'serial_port': pty_symlink,
                    'baud_rate': 115200,
                    'use_sim_time': True,
                }],
            ),
        ],
    )

    return LaunchDescription([
        rover_name_arg,
        capabilities_arg,
        module_type_arg,
        world_file_arg,
        headless_arg,
        pty_symlink_arg,
        set_gz_resource_path,
        gazebo_gui,
        gazebo_headless,
        clock_bridge,
        spawn_rover,
        rover_topic_bridge,
        sim_proxy,
        dock_bridge,
    ])
