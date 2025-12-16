#!/usr/bin/env python3
"""
COVEN Native Simulation Launch

Launches the COVEN simulation environment (Gazebo world + core infrastructure).
Rover bridges are spawned dynamically when rovers are created via ./coven menu.

Usage:
    ros2 launch coven_core coven_sim.launch.py
    ros2 launch coven_core coven_sim.launch.py headless:=true
    ros2 launch coven_core coven_sim.launch.py verbose:=true

Author: Alexander Shultis
Date: December 2025
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
    LogInfo,
    EmitEvent,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Generate launch description for COVEN simulation."""

    pkg_coven = get_package_share_directory('coven_core')

    # Paths
    default_world_file = os.path.join(pkg_coven, 'worlds', 'coven_test.sdf')
    models_path = os.path.join(pkg_coven, 'models')

    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless (no GUI)'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose Gazebo output (default: quiet mode)'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_file,
        description='Path to SDF world file'
    )

    # Set Gazebo model path to find our rover
    set_gz_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Gazebo launch command
    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')
    world_file = LaunchConfiguration('world_file')

    # Quiet mode: -v 1 (errors only), Verbose mode: -v 4 (debug)
    gazebo_quiet = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', world_file],
        output='screen',
        shell=False,
        condition=UnlessCondition(verbose),
    )

    gazebo_verbose = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        shell=False,
        condition=IfCondition(verbose),
    )

    # Note: headless mode uses -s flag for server-only
    _ = headless  # Suppress unused warning until headless is fully implemented

    # Clock bridge (Gazebo -> ROS2) - always needed
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Static transform for map->odom (SLAM will update this dynamically)
    # This provides an initial identity transform until SLAM takes over
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Static transform for odom->base_link (rover bridges will override this)
    # This provides an initial identity transform so Nav2 can start before rover spawns
    # When a rover spawns, its odom_tf_broadcaster publishes the real transform
    static_odom_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Robot state publisher (for URDF/TF tree)
    # Publishes wheel transforms from joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': '''<?xml version="1.0"?>
<robot name="coven_rover">
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>
  <link name="caster"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="-0.12 0 -0.04" rpy="0 0 0"/>
  </joint>
</robot>'''
        }],
        output='screen',
    )

    # SLAM Toolbox for mapping (LifecycleNode - requires configure + activate)
    # Waits for /scan and /odom topics (provided by rover bridges)
    # Uses proper YAML config file for correct parameter loading
    slam_params_file = os.path.join(pkg_coven, 'config', 'slam_params.yaml')
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Configure SLAM Toolbox (lifecycle transition)
    slam_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # Activate SLAM Toolbox after configuration completes
    slam_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='SLAM Toolbox configured, activating...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # NOTE: Nav2 is launched per-rover in spawn_rover_bridges() (./coven CLI)
    # Each rover gets its own namespaced Nav2 stack (e.g., /Hermione_Granger/navigate_to_pose)
    # This enables proper multi-robot navigation with isolated costmaps and planners

    # Build launch description
    ld = LaunchDescription()

    # Arguments
    ld.add_action(headless_arg)
    ld.add_action(verbose_arg)
    ld.add_action(world_file_arg)

    # Environment
    ld.add_action(set_gz_model_path)

    # Gazebo (one will run based on verbose condition)
    ld.add_action(gazebo_quiet)
    ld.add_action(gazebo_verbose)

    # Launch sequence:
    # 1. Gazebo starts immediately
    # 2. After 2s: Clock bridge + static TF + robot_state_publisher
    # 3. After 5s: SLAM (waits for /scan, /odom from rover bridges)
    #
    # NOTE: Rover-specific infrastructure (bridges, Nav2) is spawned dynamically
    # when rovers are created via ./coven menu. Each rover gets its own
    # namespaced Nav2 stack for proper multi-robot navigation.

    # Stage 1: Core infrastructure
    ld.add_action(TimerAction(period=2.0, actions=[
        clock_bridge,
        static_map_odom_tf,
        static_odom_base_tf,
        robot_state_publisher,
        LogInfo(msg="Core infrastructure started (clock, TF, robot_state_publisher)"),
        LogInfo(msg="Spawn a rover via ./coven menu to start SLAM/Nav2"),
    ]))

    # Stage 2: SLAM Toolbox (needs /scan, /odom from rover bridges)
    # Note: SLAM Toolbox is a LifecycleNode, so we need to configure + activate it
    ld.add_action(slam_activate_event)  # Register event handler first
    ld.add_action(TimerAction(period=5.0, actions=[
        slam_toolbox,
        slam_configure_event,  # Trigger configuration after node starts
        LogInfo(msg="SLAM Toolbox started - configuring and activating..."),
    ]))

    # Nav2 is launched per-rover when spawned (see ./coven spawn_rover_bridges)

    return ld
