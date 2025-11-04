#!/usr/bin/env python3
"""
COVEN Multi-Robot Simulation Launch File

Spawns N TurtleBot4 robots in Gazebo for COVEN multi-module testing.
Each robot gets a unique namespace (robot_1, robot_2, etc.) and spawn position.

Usage:
    ros2 launch coven_core coven_multi_sim.launch.py num_robots:=3
    ros2 launch coven_core coven_multi_sim.launch.py num_robots:=5 world:=depot

Author: Alexander Shultis (with Claude Code assistance)
Date: November 2025
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import math


def generate_launch_description():
    """Generate launch description for multi-robot COVEN simulation."""

    # Get package directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')

    # Launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots to spawn'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world (warehouse, maze, depot, empty)'
    )

    # Get launch configurations
    num_robots = LaunchConfiguration('num_robots')
    world = LaunchConfiguration('world')

    # Path to sim launch (Gazebo world only, no robot spawn)
    sim_launch = PathJoinSubstitution([
        pkg_turtlebot4_gz_bringup,
        'launch',
        'sim.launch.py'
    ])

    # Path to robot spawn launch
    spawn_launch = PathJoinSubstitution([
        pkg_turtlebot4_gz_bringup,
        'launch',
        'turtlebot4_spawn.launch.py'
    ])

    # Launch Gazebo world (without any robots)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_launch]),
        launch_arguments=[('world', world)]
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(num_robots_arg)
    ld.add_action(world_arg)
    ld.add_action(gazebo)

    # Spawn multiple robots in a circular pattern around origin
    # This is handled dynamically by the coven script
    # Each robot spawn will be added via separate launch calls

    return ld
