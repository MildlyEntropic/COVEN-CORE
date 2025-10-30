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

    # Config files
    nav2_params = os.path.join(pkg_coven, 'config', 'nav2_simple.yaml')
    slam_params = os.path.join(pkg_coven, 'config', 'slam_simple.yaml')

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
    # NAVIGATION STACK (optional)
    # ============================================================================

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(with_nav)
    )

    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/cmd_vel')],
        condition=IfCondition(with_nav)
    )

    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
        condition=IfCondition(with_nav)
    )

    # Nav2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        condition=IfCondition(with_nav)
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
        condition=IfCondition(with_nav)
    )

    # Nav2 Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
        condition=IfCondition(with_nav)
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        condition=IfCondition(with_nav)
    )

    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ],
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

    # Add navigation nodes (optional)
    ld.add_action(slam_node)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(lifecycle_manager)

    # Add COVEN core (always)
    ld.add_action(dock_node)
    ld.add_action(module_node)

    return ld
