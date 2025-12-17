#!/usr/bin/env python3
"""
COVEN Dock-Centric 2-Rover Simulation Launch

Demonstrates the dock-centric architecture with two rovers:
- Both rovers run simplified_module (sensors + velocity only)
- Dock runs simplified_dock with SLAM and Nav2
- Primary rover (rover1) provides sensor data for SLAM

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
    ExecuteProcess,
    SetEnvironmentVariable,
    LogInfo,
    TimerAction,
)

# Gazebo service/type strings
GZ_CREATE_SERVICE = '/world/coven_world/create'
GZ_ENTITY_FACTORY_TYPE = 'gz.msgs.EntityFactory'
GZ_BOOLEAN_TYPE = 'gz.msgs.Boolean'

# ROS argument strings
ROS_ARG_SIM_TIME = 'use_sim_time:=true'

# Fixed names for deterministic TF frames (critical for SLAM/Nav2)
ROVER1_NAME = "rover1"
ROVER2_NAME = "rover2"
DOCK_NAME = "dock1"


def generate_launch_description():
    """Generate the launch description for 2-rover dock-centric simulation."""

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

    # Positions - dock at origin, rovers around it
    dock_x, dock_y = 0.0, 0.0
    rover1_x, rover1_y = dock_x + 1.2, dock_y       # East of dock
    rover2_x, rover2_y = dock_x, dock_y + 1.2       # North of dock

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
            LogInfo(msg='[Dock-Centric-2] Starting clock bridge'),
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
            LogInfo(msg='[Dock-Centric-2] Spawning dock and 2 rovers'),
            # Spawn dock
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', GZ_CREATE_SERVICE,
                    '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                    '--reptype', GZ_BOOLEAN_TYPE,
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{dock_sdf}", name: "{DOCK_NAME}", '
                             f'pose: {{position: {{x: {dock_x}, y: {dock_y}, z: 0.05}}}}'
                ],
                output='screen',
                name='spawn_dock'
            ),
            # Spawn rover1 (primary - east of dock)
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', GZ_CREATE_SERVICE,
                    '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                    '--reptype', GZ_BOOLEAN_TYPE,
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{rover_sdf}", name: "{ROVER1_NAME}", '
                             f'pose: {{position: {{x: {rover1_x}, y: {rover1_y}, z: 0.05}}}}'
                ],
                output='screen',
                name='spawn_rover1'
            ),
            # Spawn rover2 (north of dock)
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', GZ_CREATE_SERVICE,
                    '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                    '--reptype', GZ_BOOLEAN_TYPE,
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{rover_sdf}", name: "{ROVER2_NAME}", '
                             f'pose: {{position: {{x: {rover2_x}, y: {rover2_y}, z: 0.05}}}}'
                ],
                output='screen',
                name='spawn_rover2'
            ),
        ]
    )

    # ============================================================
    # STAGE 4: Sensor Bridges for both rovers (after models spawn)
    # ============================================================
    sensor_bridges = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=f'[Dock-Centric-2] Starting sensor bridges for {ROVER1_NAME} and {ROVER2_NAME}'),
            # Rover1 Lidar bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/world/coven_world/model/{ROVER1_NAME}/link/lidar_link/sensor/lidar/scan'
                    f'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '--ros-args', '-r',
                    f'/world/coven_world/model/{ROVER1_NAME}/link/lidar_link/sensor/lidar/scan:=/{ROVER1_NAME}/scan',
                ],
                output='screen',
                name=f'lidar_bridge_{ROVER1_NAME}'
            ),
            # Rover1 Odometry bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/model/{ROVER1_NAME}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '--ros-args', '-r',
                    f'/model/{ROVER1_NAME}/odometry:=/{ROVER1_NAME}/odom',
                ],
                output='screen',
                name=f'odom_bridge_{ROVER1_NAME}'
            ),
            # Rover1 Cmd_vel bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/model/{ROVER1_NAME}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '--ros-args', '-r',
                    f'/model/{ROVER1_NAME}/cmd_vel:=/{ROVER1_NAME}/cmd_vel',
                ],
                output='screen',
                name=f'cmd_vel_bridge_{ROVER1_NAME}'
            ),
            # Rover2 Lidar bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/world/coven_world/model/{ROVER2_NAME}/link/lidar_link/sensor/lidar/scan'
                    f'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '--ros-args', '-r',
                    f'/world/coven_world/model/{ROVER2_NAME}/link/lidar_link/sensor/lidar/scan:=/{ROVER2_NAME}/scan',
                ],
                output='screen',
                name=f'lidar_bridge_{ROVER2_NAME}'
            ),
            # Rover2 Odometry bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/model/{ROVER2_NAME}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '--ros-args', '-r',
                    f'/model/{ROVER2_NAME}/odometry:=/{ROVER2_NAME}/odom',
                ],
                output='screen',
                name=f'odom_bridge_{ROVER2_NAME}'
            ),
            # Rover2 Cmd_vel bridge
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/model/{ROVER2_NAME}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '--ros-args', '-r',
                    f'/model/{ROVER2_NAME}/cmd_vel:=/{ROVER2_NAME}/cmd_vel',
                ],
                output='screen',
                name=f'cmd_vel_bridge_{ROVER2_NAME}'
            ),
        ]
    )

    # ============================================================
    # STAGE 5: TF Broadcasters for both rovers (after bridges)
    # Primary rover (rover1) uses generic frames for SLAM/Nav2
    # ============================================================
    tf_broadcasters = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[Dock-Centric-2] Starting TF broadcasters'),
            # Rover1 TF (primary - uses generic odom/base_link for SLAM)
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'odom_tf_broadcaster',
                    '--ros-args',
                    '-r', f'odom:=/{ROVER1_NAME}/odom',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', 'odom_frame:=odom',
                    '-p', 'base_frame:=base_link',
                ],
                output='screen',
                name=f'odom_tf_{ROVER1_NAME}'
            ),
            # Rover2 TF (uses namespaced frames)
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'odom_tf_broadcaster',
                    '--ros-args',
                    '-r', f'odom:=/{ROVER2_NAME}/odom',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'odom_frame:={ROVER2_NAME}/odom',
                    '-p', f'base_frame:={ROVER2_NAME}/base_link',
                ],
                output='screen',
                name=f'odom_tf_{ROVER2_NAME}'
            ),
        ]
    )

    # ============================================================
    # STAGE 6: Full Module Nodes (with bidding/task execution)
    # ============================================================
    module_nodes = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[Dock-Centric-2] Starting module nodes with COVEN protocol'),
            # Rover1 (primary)
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'module',
                    '--ros-args',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'module_id:={ROVER1_NAME}',
                    '-p', f'robot_namespace:={ROVER1_NAME}',
                    '-p', 'skip_health_check:=false',
                ],
                output='screen',
                name=f'module_{ROVER1_NAME}'
            ),
            # Rover2
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'module',
                    '--ros-args',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'module_id:={ROVER2_NAME}',
                    '-p', f'robot_namespace:={ROVER2_NAME}',
                    '-p', 'skip_health_check:=false',
                ],
                output='screen',
                name=f'module_{ROVER2_NAME}'
            ),
        ]
    )

    # ============================================================
    # STAGE 7: Full Dock Node (with bidding/task assignment)
    # ============================================================
    dock_node = TimerAction(
        period=14.0,
        actions=[
            LogInfo(msg=f'[Dock-Centric-2] Starting dock node ({DOCK_NAME}) with COVEN protocol'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'dock',
                    '--ros-args',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'dock_name:={DOCK_NAME}',
                ],
                output='screen',
                name='dock'
            ),
        ]
    )

    # ============================================================
    # STAGE 8: SLAM Toolbox (runs on dock, uses rover1's sensors)
    # ============================================================
    slam_toolbox = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg='[Dock-Centric-2] Starting SLAM Toolbox (using rover1 sensors)'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                    f'slam_params_file:={slam_params_file}',
                    'use_sim_time:=true',
                    f'scan_topic:=/{ROVER1_NAME}/scan',
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
    # STAGE 9a: Topic relays for Nav2 (rover1 is primary for SLAM)
    # ============================================================
    topic_relays = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg='[Dock-Centric-2] Starting topic relays for Nav2'),
            # Relay scan from rover1 to root for Nav2
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'topic_relay',
                    '--ros-args',
                    '-p', f'input_topic:=/{ROVER1_NAME}/scan',
                    '-p', 'output_topic:=/scan',
                    '-p', 'msg_type:=scan',
                    '-p', ROS_ARG_SIM_TIME,
                ],
                output='screen',
                name='scan_relay'
            ),
            # Relay odom from rover1 to root for Nav2
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'topic_relay',
                    '--ros-args',
                    '-p', f'input_topic:=/{ROVER1_NAME}/odom',
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
    # STAGE 9b: Nav2 (runs on dock, controls rover1 via velocity routing)
    # Rover2 is secondary and doesn't get Nav2 commands in this demo
    # ============================================================
    nav2 = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg='[Dock-Centric-2] Starting Nav2 (controlling rover1)'),
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
            # Relay cmd_vel from Nav2 to rover1
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'topic_relay',
                    '--ros-args',
                    '-p', 'input_topic:=/cmd_vel',
                    '-p', f'output_topic:=/{ROVER1_NAME}/cmd_vel',
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
        period=28.0,
        actions=[
            LogInfo(msg='[Dock-Centric-2] Starting RViz'),
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
    ld.add_action(tf_broadcasters)
    ld.add_action(module_nodes)
    ld.add_action(dock_node)
    ld.add_action(slam_toolbox)
    ld.add_action(topic_relays)
    ld.add_action(nav2)
    ld.add_action(rviz)

    return ld
