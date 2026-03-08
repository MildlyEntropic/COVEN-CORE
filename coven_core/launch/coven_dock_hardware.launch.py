#!/usr/bin/env python3
"""
coven_dock_hardware.launch.py - Launch COVEN dock on Raspberry Pi 4

Launches the dock coordination nodes:
- Frontier dispatcher (assigns exploration targets to rovers)
- Offline SLAM processor (processes recorded sensor data)
- SLAM Toolbox (builds maps from replayed data)

The dock doesn't need motor drivers or LiDAR - it's stationary and
coordinates the rovers via ROS2 topics.

Usage:
    ros2 launch coven_core coven_dock_hardware.launch.py
    ros2 launch coven_core coven_dock_hardware.launch.py coven_name:=The_Weird_Sisters

Author: Alexander Shultis
Date: January 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get package path for config files
    from ament_index_python.packages import get_package_share_directory
    pkg_path = get_package_share_directory('coven_core')
    config_path = os.path.join(pkg_path, 'config')
    slam_params_file = os.path.join(config_path, 'slam_params_hw.yaml')

    # ========================
    # Launch Arguments
    # ========================

    coven_name_arg = DeclareLaunchArgument(
        'coven_name',
        default_value='The_Graeae',
        description='Coven/dock name'
    )

    dock_x_arg = DeclareLaunchArgument(
        'dock_x',
        default_value='0.0',
        description='Dock X position in map frame (meters)'
    )

    dock_y_arg = DeclareLaunchArgument(
        'dock_y',
        default_value='0.0',
        description='Dock Y position in map frame (meters)'
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='~/Desktop/COVEN/Data',
        description='Directory to watch for incoming rover data'
    )

    # Get launch configurations
    coven_name = LaunchConfiguration('coven_name')
    dock_x = LaunchConfiguration('dock_x')
    dock_y = LaunchConfiguration('dock_y')
    data_dir = LaunchConfiguration('data_dir')

    # ========================
    # Rover Bridge
    # ========================

    # Bridges Rust rovers to ROS2 via serial UART connection.
    # Manages COVEN handshake, translates sensor data to ROS2 topics,
    # and dispatches tasks to rovers via the auctioneer.
    rover_bridge_node = Node(
        package='coven_core',
        executable='rover_bridge',
        name='rover_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'coven_name': coven_name,
            'dock_x': dock_x,
            'dock_y': dock_y,
            'data_base_dir': data_dir,
        }],
    )

    # ========================
    # Frontier Dispatcher
    # ========================

    # Coordinates rover exploration by detecting frontiers in the map
    # and dispatching rovers to explore them
    dispatcher_node = Node(
        package='coven_core',
        executable='frontier_dispatcher',
        name='frontier_dispatcher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'dock_position_x': dock_x,
            'dock_position_y': dock_y,
            # Frontier detection parameters
            'min_frontier_size': 10,          # Minimum cells to consider frontier
            'exploration_radius': 5.0,        # meters — how far to send rovers
            'coverage_goal': 0.8,             # 80% coverage target
        }],
    )

    # ========================
    # Offline SLAM Processor
    # ========================

    # Watches for incoming data files from rovers and replays them
    # through SLAM Toolbox to build/update the map
    slam_processor_node = Node(
        package='coven_core',
        executable='offline_slam_processor',
        name='offline_slam_processor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'data_dir': data_dir,
            # Replay parameters
            'playback_speed': 10.0,           # Replay at 10x real-time
            'scan_topic': '/offline_scan',    # Topic for replayed scans
            'odom_topic': '/offline_odom',    # Topic for replayed odom
        }],
    )

    # ========================
    # SLAM Toolbox
    # ========================

    # Async SLAM - processes scans as they come in (from replay)
    # Builds occupancy grid map
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': False,
                # Override scan topic to use offline replay topic
                'scan_topic': '/offline_scan',
            }
        ],
        remappings=[
            # Remap to use offline topics during replay
            ('/scan', '/offline_scan'),
        ],
    )

    # ========================
    # Static Transform: map -> odom
    # ========================

    # For offline processing, we assume map = odom (no localization drift)
    # In a real multi-session scenario, you'd use AMCL or similar
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map', 'odom'
        ],
    )

    # ========================
    # Launch Description
    # ========================

    return LaunchDescription([
        # Arguments
        coven_name_arg,
        dock_x_arg,
        dock_y_arg,
        data_dir_arg,

        # Log startup info
        LogInfo(msg=['Launching COVEN dock: ', coven_name]),
        LogInfo(msg=['Data directory: ', data_dir]),

        # Nodes
        rover_bridge_node,
        dispatcher_node,
        slam_processor_node,
        slam_node,
        static_tf_map_odom,
    ])
