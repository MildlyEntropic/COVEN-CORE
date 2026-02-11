#!/usr/bin/env python3
"""
coven_rover_hardware.launch.py - Launch hardware drivers for COVEN rover

NOTE: This launch file is for the OLD Python-based rover architecture.
The new architecture uses Rust rovers communicating via TCP:

  New Architecture:
  - Rust rover binary (not launched via ROS2)
  - rover_bridge.py on dock handles TCP connections

  This file launches ROS2 hardware drivers only:
  - YDLiDAR driver (publishes /scan)
  - Motor driver (subscribes /cmd_vel)
  - Encoder odometry (publishes /odom)
  - Static TF (base_link -> laser)

Usage:
    ros2 launch coven_core coven_rover_hardware.launch.py
    ros2 launch coven_core coven_rover_hardware.launch.py rover_name:=Circe

Author: Alexander Shultis
Date: January 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get package path for config files
    pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    config_path = os.path.join(pkg_path, 'config')

    # ========================
    # Launch Arguments
    # ========================

    rover_name_arg = DeclareLaunchArgument(
        'rover_name',
        default_value='Morrigan',
        description='Rover name (witch name for identification)'
    )

    coven_name_arg = DeclareLaunchArgument(
        'coven_name',
        default_value='The_Graeae',
        description='Coven/dock name this rover belongs to'
    )

    use_ydlidar_arg = DeclareLaunchArgument(
        'use_ydlidar',
        default_value='true',
        description='Launch YDLiDAR driver (set false if using different LiDAR)'
    )

    use_encoders_arg = DeclareLaunchArgument(
        'use_encoders',
        default_value='true',
        description='Use wheel encoders for odometry (false = open-loop)'
    )

    # Robot geometry parameters
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.1',
        description='Distance between wheels (meters)'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.03',
        description='Wheel radius (meters)'
    )

    # Get launch configurations
    rover_name = LaunchConfiguration('rover_name')
    coven_name = LaunchConfiguration('coven_name')
    use_ydlidar = LaunchConfiguration('use_ydlidar')
    use_encoders = LaunchConfiguration('use_encoders')
    wheel_base = LaunchConfiguration('wheel_base')
    wheel_radius = LaunchConfiguration('wheel_radius')

    # ========================
    # YDLiDAR Driver
    # ========================

    # YDLiDAR X4 parameters
    ydlidar_params = {
        'port': '/dev/ydlidar',
        'frame_id': 'laser',
        'ignore_array': '',
        'baudrate': 128000,
        'lidar_type': 1,           # X4 = TYPE_TRIANGLE
        'device_type': 0,          # YDLIDAR_TYPE_SERIAL
        'sample_rate': 5,          # 5K samples/sec
        'abnormal_check_count': 4,
        'fixed_resolution': True,
        'reversion': False,
        'inverted': True,
        'auto_reconnect': True,
        'isSingleChannel': False,
        'intensity': False,
        'support_motor_dtr': True,
        'angle_max': 180.0,
        'angle_min': -180.0,
        'range_max': 10.0,
        'range_min': 0.12,
        'frequency': 7.0,          # 7Hz scan rate
        'invalid_range_is_inf': False,
    }

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar',
        output='screen',
        parameters=[ydlidar_params],
        condition=IfCondition(use_ydlidar),
    )

    # ========================
    # Motor Driver
    # ========================

    motor_node = Node(
        package='coven_core',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[{
            'robot_namespace': '',
            'wheel_base': wheel_base,
            'wheel_radius': wheel_radius,
            'max_rpm': 100,
            'pwm_frequency': 1000,
            'cmd_timeout': 0.5,
            'invert_left': False,
            'invert_right': False,
            # GPIO pins (TB6612FNG)
            'pin_pwma': 12,
            'pin_ain1': 5,
            'pin_ain2': 6,
            'pin_pwmb': 13,
            'pin_bin1': 16,
            'pin_bin2': 26,
            'pin_stby': 17,
        }],
    )

    # ========================
    # Encoder Odometry
    # ========================

    encoder_node = Node(
        package='coven_core',
        executable='encoder_odom',
        name='encoder_odom',
        output='screen',
        parameters=[{
            'robot_namespace': '',
            'wheel_base': wheel_base,
            'wheel_radius': wheel_radius,
            'encoder_ppr': 210,       # N20 with 30:1 gearbox, 7 PPR encoder
            'publish_rate': 50.0,
            'publish_tf': True,
            'invert_left': False,
            'invert_right': False,
            # GPIO pins (encoder channels)
            'pin_left_a': 23,
            'pin_left_b': 24,
            'pin_right_a': 27,
            'pin_right_b': 22,
        }],
        condition=IfCondition(use_encoders),
    )

    # ========================
    # Static Transforms
    # ========================

    # base_link -> laser (LiDAR mounted on top of rover)
    # Adjust z offset based on your actual mount height
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '0', '0', '0.05',       # x, y, z offset (5cm up)
            '0', '0', '0',          # roll, pitch, yaw
            'base_link', 'laser'
        ],
    )

    # ========================
    # Launch Description
    # ========================

    # NOTE: The Rust rover binary handles the COVEN protocol and is launched
    # separately, not through this ROS2 launch file. This only launches
    # the hardware drivers that the Rust rover communicates with.

    return LaunchDescription([
        # Arguments
        rover_name_arg,
        coven_name_arg,
        use_ydlidar_arg,
        use_encoders_arg,
        wheel_base_arg,
        wheel_radius_arg,

        # Log startup info
        LogInfo(msg=['Launching COVEN rover hardware drivers: ', rover_name]),
        LogInfo(msg=['Coven: ', coven_name]),

        # Hardware driver nodes
        ydlidar_node,
        motor_node,
        encoder_node,
        static_tf_base_laser,
    ])
