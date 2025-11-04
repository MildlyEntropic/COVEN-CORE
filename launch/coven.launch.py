"""
coven.launch.py — COVEN Universal Launcher

Single flexible launch file for all COVEN configurations:
- Basic: Just dock + module (no nav, no sim)
- Sim: Add Create3 simulation
- Nav: Add navigation stack (SLAM + Nav2)
- Full: Everything (sim + nav + COVEN)

Usage:
    ros2 launch coven_core coven.launch.py                    # Basic: dock + module only
    ros2 launch coven_core coven.launch.py with_sim:=true     # With simulation
    ros2 launch coven_core coven.launch.py with_nav:=true     # With navigation
    ros2 launch coven_core coven.launch.py with_sim:=true with_nav:=true  # Full stack

Author: Alexander Shultis
Date: October 2025
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description based on arguments."""

    pkg_coven = get_package_share_directory('coven_core')

    # Launch arguments
    with_sim = LaunchConfiguration('with_sim')
    with_nav = LaunchConfiguration('with_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Declare arguments
    declare_with_sim = DeclareLaunchArgument(
        'with_sim',
        default_value='false',
        description='Launch Create3 Gazebo simulation'
    )

    declare_with_nav = DeclareLaunchArgument(
        'with_nav',
        default_value='false',
        description='Launch navigation stack (SLAM + Nav2)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world (warehouse, maze, depot, empty)'
    )

    # Package directories
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    # ============================================================================
    # SIMULATION (optional)
    # ============================================================================
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('irobot_create_gz_bringup'),
                'launch',
                'create3_gz.launch.py'
            ])
        ),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(with_sim)
    )

    # ============================================================================
    # NAVIGATION STACK (optional) - Use TurtleBot4 launch files
    # ============================================================================

    # SLAM - Use TurtleBot4's SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sync': 'true',  # Use sync SLAM for better performance
            'use_lifecycle_manager': 'false',  # SLAM handles its own lifecycle
        }.items(),
        condition=IfCondition(with_nav)
    )

    # Nav2 - Use TurtleBot4's Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(with_nav)
    )

    # ============================================================================
    # COVEN CORE (always included)
    # ============================================================================

    # COVEN Dock (delayed start if sim/nav present)
    dock_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='coven_core',
                executable='dock_multi',
                name='coven_dock',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # COVEN Module (delayed start if nav present)
    module_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='coven_core',
                executable='module',
                name='coven_module',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # ============================================================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================================================

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_with_sim)
    ld.add_action(declare_with_nav)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)

    # Add simulation (optional)
    ld.add_action(simulation)

    # Add navigation stack (optional) - using TurtleBot4 launch files
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)

    # Add COVEN core (always)
    ld.add_action(dock_node)
    ld.add_action(module_node)

    return ld
