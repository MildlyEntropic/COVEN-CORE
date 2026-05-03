#!/usr/bin/env python3
"""
COVEN Dock-Centric Simulation Launch

Demonstrates the dock-centric architecture where:
- Rovers run simplified_module (sensors + velocity only)
- Dock runs simplified_dock with SLAM and Nav2

Key differences from standard simulation:
- No Nav2/SLAM on rovers - they just publish sensor data
- Dock receives sensor data and republishes to standard topics
- Dock runs SLAM Toolbox and Nav2 for centralized planning
- Velocity commands flow from dock to rovers

Author: Alexander Shultis
Date: December 2025
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

# Gazebo service/type strings
GZ_CREATE_SERVICE = '/world/coven_world/create'
GZ_ENTITY_FACTORY_TYPE = 'gz.msgs.EntityFactory'
GZ_BOOLEAN_TYPE = 'gz.msgs.Boolean'

# ROS argument strings
ROS_ARG_SIM_TIME = 'use_sim_time:=true'

# Fixed names for deterministic TF frames (critical for SLAM/Nav2)
# Use simple names without underscores to avoid TF namespace issues
DEFAULT_ROVER_NAME = "rover1"
DEFAULT_DOCK_NAME = "dock1"


def generate_launch_description():
    """Generate the launch description for dock-centric simulation."""

    # Package paths
    pkg_coven = get_package_share_directory('coven_core')
    models_path = os.path.join(pkg_coven, 'models')
    worlds_path = os.path.join(pkg_coven, 'worlds')

    # Config files
    slam_params_file = os.path.join(pkg_coven, 'config', 'slam_params_sim.yaml')
    nav2_params_file = os.path.join(pkg_coven, 'config', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(pkg_coven, 'config', 'coven_sim.rviz')
    rover_sdf = os.path.join(models_path, 'coven_rover', 'model.sdf')
    dock_sdf = os.path.join(models_path, 'dock.sdf')
    world_sdf = os.path.join(worlds_path, 'coven_4rover.sdf')

    # Positions
    dock_x, dock_y = 0.0, 0.0
    rover_x = dock_x + 1.2
    rover_y = dock_y
    rover_name = DEFAULT_ROVER_NAME
    coven_name = DEFAULT_DOCK_NAME

    # Environment
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{worlds_path}'
    )

    # ============================================================
    # STAGE 1: Gazebo
    # ============================================================
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', world_sdf
        ],
        output='screen',
        name='gazebo'
    )

    # ============================================================
    # STAGE 2: Clock Bridge (after Gazebo starts)
    # ============================================================
    clock_bridge = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[Dock-Centric] Starting clock bridge'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                output='screen',
                name='clock_bridge'
            ),
        ]
    )

    # ============================================================
    # STAGE 3: Spawn Models
    # ============================================================
    spawn_models = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[Dock-Centric] Spawning dock and rover'),
            # Spawn dock
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', GZ_CREATE_SERVICE,
                    '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                    '--reptype', GZ_BOOLEAN_TYPE,
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{dock_sdf}", name: "{coven_name}", '
                             f'pose: {{position: {{x: {dock_x}, y: {dock_y}, z: 0.05}}}}'
                ],
                output='screen',
                name='spawn_dock'
            ),
            # Spawn rover
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', GZ_CREATE_SERVICE,
                    '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                    '--reptype', GZ_BOOLEAN_TYPE,
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{rover_sdf}", name: "{rover_name}", '
                             f'pose: {{position: {{x: {rover_x}, y: {rover_y}, z: 0.05}}}}'
                ],
                output='screen',
                name='spawn_rover'
            ),
        ]
    )

    # ============================================================
    # STAGE 4: Sensor Bridges (after models spawn)
    # ============================================================
    sensor_bridges = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=f'[Dock-Centric] Starting sensor bridges for {rover_name}'),
            # Lidar bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/world/coven_world/model/{rover_name}/link/lidar_link/sensor/lidar/scan'
                    f'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '--ros-args', '-r',
                    f'/world/coven_world/model/{rover_name}/link/lidar_link/sensor/lidar/scan:=/{rover_name}/scan',
                ],
                output='screen',
                name=f'lidar_bridge_{rover_name}'
            ),
            # Odometry bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/model/{rover_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '--ros-args', '-r',
                    f'/model/{rover_name}/odometry:=/{rover_name}/odom',
                ],
                output='screen',
                name=f'odom_bridge_{rover_name}'
            ),
            # Cmd_vel bridge (Gazebo listens to this)
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/model/{rover_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '--ros-args', '-r',
                    f'/model/{rover_name}/cmd_vel:=/{rover_name}/cmd_vel',
                ],
                output='screen',
                name=f'cmd_vel_bridge_{rover_name}'
            ),
        ]
    )

    # ============================================================
    # STAGE 5: TF Broadcaster (after bridges)
    # Note: Gazebo diff-drive plugin publishes frames as "odom" and "base_link"
    # (not namespaced), so we use these generic frames for SLAM/Nav2 compatibility.
    # For multi-rover, each rover would need a modified SDF with unique frame names.
    # ============================================================
    tf_broadcaster = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg=f'[Dock-Centric] Starting TF broadcaster for {rover_name}'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'odom_tf_broadcaster',
                    '--ros-args',
                    '-r', f'odom:=/{rover_name}/odom',
                    '-p', ROS_ARG_SIM_TIME,
                    # Use generic frames matching Gazebo plugin output
                    '-p', 'odom_frame:=odom',
                    '-p', 'base_frame:=base_link',
                ],
                output='screen',
                name=f'odom_tf_{rover_name}'
            ),
        ]
    )

    # ============================================================
    # STAGE 6: Full Module Node (with bidding/task execution)
    # ============================================================
    module_node = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=f'[Dock-Centric] Starting module node ({rover_name}) with COVEN protocol'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'module',
                    '--ros-args',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'module_id:={rover_name}',
                    '-p', f'robot_namespace:={rover_name}',
                    '-p', 'skip_health_check:=false',  # Enable health checks in sim
                ],
                output='screen',
                name=f'module_{rover_name}'
            ),
        ]
    )

    # ============================================================
    # STAGE 7: Full Dock Node (with bidding/task assignment)
    # ============================================================
    dock_node = TimerAction(
        period=14.0,
        actions=[
            LogInfo(msg=f'[Dock-Centric] Starting dock node ({coven_name}) with COVEN protocol'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'dock',
                    '--ros-args',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'dock_name:={coven_name}',
                ],
                output='screen',
                name='dock'
            ),
        ]
    )

    # ============================================================
    # STAGE 8: SLAM Toolbox (runs on dock, uses rover sensors)
    # Uses generic odom/base_link frames (from Gazebo) and rover's scan topic
    # ============================================================
    slam_toolbox = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg='[Dock-Centric] Starting SLAM Toolbox (centralized on dock)'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                    f'slam_params_file:={slam_params_file}',
                    'use_sim_time:=true',
                    # Use rover's scan topic but generic frames from Gazebo
                    f'scan_topic:=/{rover_name}/scan',
                    'base_frame:=base_link',
                    'odom_frame:=odom',
                    'map_frame:=map',
                ],
                output='screen',
                name='slam_toolbox'
            ),
        ]
    )

    # ============================================================
    # STAGE 9a: Topic relays for Nav2
    # Nav2 expects /scan and /odom at root, rover publishes to namespaced topics
    # ============================================================
    topic_relays = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg='[Dock-Centric] Starting topic relays for Nav2'),
            # Relay scan from rover to root for Nav2
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'topic_relay',
                    '--ros-args',
                    '-p', f'input_topic:=/{rover_name}/scan',
                    '-p', 'output_topic:=/scan',
                    '-p', 'msg_type:=scan',
                    '-p', ROS_ARG_SIM_TIME,
                ],
                output='screen',
                name='scan_relay'
            ),
            # Relay odom from rover to root for Nav2
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'topic_relay',
                    '--ros-args',
                    '-p', f'input_topic:=/{rover_name}/odom',
                    '-p', 'output_topic:=/odom',
                    '-p', 'msg_type:=odom',
                    '-p', ROS_ARG_SIM_TIME,
                ],
                output='screen',
                name='odom_relay'
            ),
        ]
    )

    # ============================================================
    # STAGE 9b: Nav2 (runs on dock, controls rover via velocity routing)
    # Note: Don't namespace Nav2 - use global odom/base_link frames
    # Nav2 outputs to /cmd_vel, we relay to /{rover_name}/cmd_vel
    # ============================================================
    nav2 = TimerAction(
        period=22.0,  # After relays are up
        actions=[
            LogInfo(msg='[Dock-Centric] Starting Nav2 (centralized on dock)'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                    f'params_file:={nav2_params_file}',
                    'use_sim_time:=true',
                    'autostart:=true',
                ],
                output='screen',
                name='nav2'
            ),
            # Relay cmd_vel from Nav2 to rover
            # Nav2 publishes to /cmd_vel, rover listens on /{rover_name}/cmd_vel
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'topic_relay',
                    '--ros-args',
                    '-p', 'input_topic:=/cmd_vel',
                    '-p', f'output_topic:=/{rover_name}/cmd_vel',
                    '-p', 'msg_type:=twist',
                    '-p', ROS_ARG_SIM_TIME,
                ],
                output='screen',
                name='cmd_vel_relay'
            ),
        ]
    )

    # ============================================================
    # STAGE 10: RViz
    # ============================================================
    rviz = TimerAction(
        period=28.0,  # After Nav2 starts
        actions=[
            LogInfo(msg='[Dock-Centric] Starting RViz'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'rviz2', 'rviz2',
                    '-d', rviz_config,
                ],
                output='screen',
                name='rviz2'
            ),
        ]
    )

    # ============================================================
    # Build Launch Description
    # ============================================================
    ld = LaunchDescription()

    # Environment
    ld.add_action(gz_model_path)

    # Launch sequence
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_models)
    ld.add_action(sensor_bridges)
    ld.add_action(tf_broadcaster)
    ld.add_action(module_node)
    ld.add_action(dock_node)
    ld.add_action(slam_toolbox)
    ld.add_action(topic_relays)
    ld.add_action(nav2)
    ld.add_action(rviz)

    return ld
