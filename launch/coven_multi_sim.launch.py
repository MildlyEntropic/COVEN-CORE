#!/usr/bin/env python3
"""
COVEN Multi-Robot Simulation Launch File

Spawns N TurtleBot4 robots in Gazebo for COVEN multi-module testing.
Each robot gets:
- Unique namespace (robot_1, robot_2, etc.)
- Unique spawn position (circular pattern around origin)
- SLAM and Nav2 stack
- COVEN module node with matching namespace

Usage:
    ros2 launch coven_core coven_multi_sim.launch.py num_robots:=1
    ros2 launch coven_core coven_multi_sim.launch.py num_robots:=3
    ros2 launch coven_core coven_multi_sim.launch.py num_robots:=3 world:=depot

Author: Alexander Shultis
Date: November 2025
"""

import math
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_robot_spawns(context, *args, **kwargs):
    """
    Dynamically generate robot spawn actions based on num_robots parameter.

    This function is called at launch time when the actual parameter value is known.
    """
    # Get parameter values from context
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    # Get package directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_coven_core = get_package_share_directory('coven_core')

    # Path to robot spawn launch
    spawn_launch_path = PathJoinSubstitution([
        pkg_turtlebot4_gz_bringup,
        'launch',
        'turtlebot4_spawn.launch.py'
    ]).perform(context)

    actions = []

    # Spawn positions in circular pattern around origin
    # Radius increases slightly with more robots to avoid collisions
    radius = 2.0 + (num_robots * 0.5)

    for i in range(num_robots):
        robot_ns = f'robot_{i + 1}'

        # Calculate spawn position in circular pattern
        angle = (2 * math.pi * i) / max(num_robots, 1)
        spawn_x = radius * math.cos(angle)
        spawn_y = radius * math.sin(angle)
        # Face toward center
        spawn_yaw = angle + math.pi

        # Delay each robot spawn to give Gazebo time to process
        # First robot: 5s after Gazebo, subsequent: +10s each
        spawn_delay = 5.0 + (i * 10.0)

        # Spawn TurtleBot4 with SLAM and Nav2
        spawn_robot = TimerAction(
            period=spawn_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_launch_path),
                    launch_arguments=[
                        ('namespace', robot_ns),
                        ('x', str(spawn_x)),
                        ('y', str(spawn_y)),
                        ('yaw', str(spawn_yaw)),
                        ('slam', 'true'),
                        ('nav2', 'true'),
                        ('use_sim_time', 'true'),
                    ]
                ),
            ]
        )
        actions.append(spawn_robot)

        # Spawn COVEN module for this robot
        # Delay module start to allow robot sensors to initialize
        module_delay = spawn_delay + 8.0  # 8s after robot spawn

        coven_module = TimerAction(
            period=module_delay,
            actions=[
                Node(
                    package='coven_core',
                    executable='module',
                    name=f'coven_module_{robot_ns}',  # Unique node name
                    # NOTE: No namespace here! COVEN protocol topics must be global
                    # so dock can communicate with all modules on /coven/* topics.
                    # The robot_namespace param tells the module which robot's
                    # sensors/nav to use (e.g., /robot_1/scan, /robot_1/map)
                    parameters=[{
                        'robot_namespace': robot_ns,
                        'skip_health_check': False,
                        'use_sim_time': True,
                    }],
                    output='screen',
                    emulate_tty=True,
                ),
            ]
        )
        actions.append(coven_module)

    return actions


def generate_launch_description():
    """Generate launch description for multi-robot COVEN simulation."""

    # Get package directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')

    # Launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn (1-5 recommended)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world (warehouse, maze, depot)'
    )

    # Path to sim launch (Gazebo world only, no robot)
    sim_launch = PathJoinSubstitution([
        pkg_turtlebot4_gz_bringup,
        'launch',
        'sim.launch.py'
    ])

    # Launch Gazebo world first
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_launch]),
        launch_arguments=[('world', LaunchConfiguration('world'))]
    )

    # COVEN Dock node - starts after Gazebo is up
    # Dock doesn't need a namespace - it's the coordinator
    dock_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='coven_core',
                executable='dock',
                name='coven_dock',
                parameters=[{'use_sim_time': True}],
                output='screen',
                emulate_tty=True,
            ),
        ]
    )

    # Build launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(num_robots_arg)
    ld.add_action(world_arg)

    # Start Gazebo
    ld.add_action(gazebo)

    # Start COVEN dock
    ld.add_action(dock_node)

    # Dynamically spawn robots based on num_robots parameter
    ld.add_action(OpaqueFunction(function=generate_robot_spawns))

    return ld
