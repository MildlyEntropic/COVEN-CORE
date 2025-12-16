#!/usr/bin/env python3
"""
COVEN 4-Rover Simulation Launch (True Event-Driven)

Full simulation environment with:
- 1 dock (configurable position)
- 4 rovers arranged around the dock
- All bridges, TF, and Nav2 stacks pre-configured
- COVEN protocol nodes for each rover

Each rover runs in its own gnome-terminal with internal topic-based waits
for true event-driven startup.

Author: Alexander Shultis
Date: December 2025
"""

import os
import math
import random
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
from launch.conditions import IfCondition, UnlessCondition

# Common ROS2 argument strings
ROS_ARG_SIM_TIME = 'use_sim_time:=true'
ROS_ARG_NS_PREFIX = '__ns:=/'


# Full witch name list from coven_core.common
WITCH_NAMES = [
    "Morgan_Le_Fay", "Hecate", "Circe", "Scathach", "Morrigan",
    "Lorelei", "Frau_Holle", "Louhi", "Baba_Yaga", "Mami_Wata",
    "Princess_Kaguya", "Elphaba", "Glinda", "Wanda_Maximoff", "Agatha_Harkness",
    "Zatanna_Zatara", "Hermione_Granger", "Minerva_McGonagall", "Sabrina_Spellman",
    "Willow_Rosenberg", "Maleficent", "Endora", "Samantha_Stephens",
    "Kiki", "Yubaba", "Akko", "Nico_Wakatsuki",
    "Mother_Talzin", "Old_Daka", "Axkva_Min",
]

COVEN_NAMES = [
    "The_Graeae", "The_Erinyes", "The_Norns", "The_Weird_Sisters",
    "The_Sanderson_Sisters", "The_Hex_Girls", "The_Crones",
    "The_Hags_of_Dun_Broch", "The_Bene_Gesserit", "The_Lilim",
]

# Rover positions relative to dock (distance, angle in radians)
ROVER_POSITIONS = [
    (1.2, math.pi / 2),      # North (front)
    (1.2, -math.pi / 2),     # South (back)
    (1.2, 0),                # East (right)
    (1.2, math.pi),          # West (left)
]

# Obstacle configuration
OBSTACLE_TYPES = [
    ('box', (0.5, 1.5), 0.8),
    ('box', (1.0, 2.5), 1.0),
    ('cylinder', (0.3, 0.8), 1.0),
]

INTERIOR_WALL_CONFIG = ((4.0, 8.0), 1.0)

OBSTACLE_ZONES = [
    (-11, -6, -11, 11),
    (6, 11, -11, 11),
    (-6, 6, -11, -6),
    (-6, 6, 6, 11),
]

SAFE_RADIUS_FROM_ORIGIN = 5.0

# Gazebo service constants
GZ_CREATE_SERVICE = '/world/coven_world/create'
GZ_ENTITY_FACTORY_TYPE = 'gz.msgs.EntityFactory'
GZ_BOOLEAN_TYPE = 'gz.msgs.Boolean'

# Generate random names at module load time
_selected_witches = random.sample(WITCH_NAMES, 4)
_selected_coven = random.choice(COVEN_NAMES)


def _is_valid_obstacle_position(x, y, used_positions):
    """Check if position is valid (far from origin and other obstacles)."""
    if math.sqrt(x**2 + y**2) < SAFE_RADIUS_FROM_ORIGIN:
        return False
    for px, py in used_positions:
        if math.sqrt((x - px)**2 + (y - py)**2) < 3.0:
            return False
    return True


def generate_random_obstacles():
    """Generate semi-random obstacle configurations."""
    obstacles = []
    used_positions = []

    for i in range(5):
        obs_type, size_range, height = random.choice(OBSTACLE_TYPES)
        zone = random.choice(OBSTACLE_ZONES)
        x_min, x_max, y_min, y_max = zone

        for _ in range(10):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            if _is_valid_obstacle_position(x, y, used_positions):
                break

        used_positions.append((x, y))
        yaw = random.uniform(0, 2 * math.pi)
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        if obs_type == 'box':
            size = random.uniform(*size_range)
            width = size * random.uniform(0.5, 1.5)
            depth = size * random.uniform(0.5, 1.5)
            geometry_sdf = f'<box><size>{width} {depth} {height}</size></box>'
            z = height / 2
        else:
            radius = random.uniform(*size_range)
            geometry_sdf = f'<cylinder><radius>{radius}</radius><length>{height}</length></cylinder>'
            z = height / 2

        obstacles.append((f'obstacle_{i+1}', geometry_sdf, x, y, z, 0, 0, qz, qw))

    return obstacles


def generate_random_interior_walls():
    """Generate 2 partial interior walls with semi-random placement."""
    walls = []
    length_range, height = INTERIOR_WALL_CONFIG

    length1 = random.uniform(*length_range)
    x1 = random.uniform(-5, 5)
    y1 = random.uniform(7, 10)
    yaw1 = random.uniform(-math.pi/6, math.pi/6)
    qz1 = math.sin(yaw1 / 2)
    qw1 = math.cos(yaw1 / 2)
    walls.append(('interior_wall_1', length1, x1, y1, height/2, 0, 0, qz1, qw1))

    length2 = random.uniform(*length_range)
    x2 = random.choice([-1, 1]) * random.uniform(7, 10)
    y2 = random.uniform(-5, 5)
    yaw2 = math.pi/2 + random.uniform(-math.pi/6, math.pi/6)
    qz2 = math.sin(yaw2 / 2)
    qw2 = math.cos(yaw2 / 2)
    walls.append(('interior_wall_2', length2, x2, y2, height/2, 0, 0, qz2, qw2))

    return walls


# Pre-generate obstacles and walls at module load time
_random_obstacles = generate_random_obstacles()
_random_interior_walls = generate_random_interior_walls()


def write_slam_config(rover_name: str) -> str:
    """Write SLAM config file for a rover and return the path."""
    config_path = f'/tmp/slam_{rover_name}.yaml'
    config_content = f"""slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
    odom_frame: {rover_name}/odom
    map_frame: map
    base_frame: {rover_name}/base_link
    scan_topic: /{rover_name}/scan
    use_map_saver: true
    mode: mapping
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
"""
    with open(config_path, 'w') as f:
        f.write(config_content)
    return config_path


def write_rover_launch_script(
    rover_name: str,
    slam_config: str,
    gz_odom: str, gz_tf: str, gz_scan: str, gz_cmd_vel: str, gz_joint: str,
    ros_odom: str, ros_cmd_vel: str, ros_joint: str,
    nav2_params_file: str,
) -> str:
    """
    Write a bash script that launches all rover nodes in sequence.
    This script uses 'ros2 topic list' to wait for topics before proceeding.
    Returns the path to the script.
    """
    script_path = f'/tmp/launch_{rover_name}.sh'
    script_content = f'''#!/bin/bash
# Auto-generated launch script for rover {rover_name}
# This script launches nodes sequentially, waiting for required topics

set -e  # Exit on error

echo "=== Starting {rover_name} Stack ==="

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Function to wait for a topic
wait_for_topic() {{
    local topic="$1"
    local timeout="${{2:-60}}"
    local count=0
    echo "Waiting for topic $topic..."
    while ! ros2 topic list 2>/dev/null | grep -q "^$topic$"; do
        sleep 0.5
        count=$((count + 1))
        if [ $count -ge $((timeout * 2)) ]; then
            echo "WARNING: Timeout waiting for $topic"
            return 1
        fi
    done
    echo "Topic $topic is available"
    return 0
}}

# Function to wait for scan data (topic must have publishers)
wait_for_scan_data() {{
    local topic="$1"
    local timeout="${{2:-60}}"
    local count=0
    echo "Waiting for scan data on $topic..."
    while true; do
        local info=$(ros2 topic info "$topic" 2>/dev/null || echo "")
        if echo "$info" | grep -q "Publisher count: [1-9]"; then
            echo "Scan data available on $topic"
            return 0
        fi
        sleep 0.5
        count=$((count + 1))
        if [ $count -ge $((timeout * 2)) ]; then
            echo "WARNING: Timeout waiting for scan data on $topic"
            return 1
        fi
    done
}}

# Wait for clock (Gazebo must be running)
echo "[1/8] Waiting for /clock from Gazebo..."
wait_for_topic "/clock" 120

# Start Gazebo bridge for this rover
echo "[2/8] Starting Gazebo bridge for {rover_name}..."
ros2 run ros_gz_bridge parameter_bridge \\
    {gz_odom}@nav_msgs/msg/Odometry[gz.msgs.Odometry \\
    {gz_tf}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
    {gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \\
    {gz_cmd_vel}@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    {gz_joint}@sensor_msgs/msg/JointState[gz.msgs.Model \\
    --ros-args \\
    -r {gz_odom}:={ros_odom} \\
    -r {gz_tf}:=/{rover_name}/tf \\
    -r {gz_scan}:=/{rover_name}/scan_raw \\
    -r {gz_cmd_vel}:={ros_cmd_vel} \\
    -r {gz_joint}:={ros_joint} \\
    -p {ROS_ARG_SIM_TIME} &
GZ_BRIDGE_PID=$!
sleep 2

# Wait for scan_raw topic
wait_for_topic "/{rover_name}/scan_raw" 30

# Start scan republisher
echo "[3/8] Starting scan frame republisher..."
ros2 run coven_core scan_frame_republisher \\
    --ros-args \\
    -p input_topic:=/{rover_name}/scan_raw \\
    -p output_topic:=/{rover_name}/scan \\
    -p frame_id:={rover_name}/base_link \\
    -p {ROS_ARG_SIM_TIME} &
SCAN_PID=$!
sleep 1

# Wait for scan topic with data
wait_for_scan_data "/{rover_name}/scan" 30

# Start odom TF broadcaster
echo "[4/8] Starting odom TF broadcaster..."
ros2 run coven_core odom_tf_broadcaster \\
    --ros-args \\
    -r __ns:=/{rover_name} \\
    -r /tf:=/tf \\
    -r /tf_static:=/tf_static \\
    -p {ROS_ARG_SIM_TIME} \\
    -p odom_topic:={ros_odom} \\
    -p odom_frame:={rover_name}/odom \\
    -p base_frame:={rover_name}/base_link &
ODOM_TF_PID=$!
sleep 1

# Start static TF (map -> odom, will be overwritten by SLAM)
echo "[5/8] Starting initial static TF publisher (map -> {rover_name}/odom)..."
ros2 run tf2_ros static_transform_publisher \\
    --frame-id map \\
    --child-frame-id {rover_name}/odom \\
    --x 0 --y 0 --z 0 \\
    --roll 0 --pitch 0 --yaw 0 \\
    --ros-args \\
    -r /tf:=/tf \\
    -r /tf_static:=/tf_static \\
    -p {ROS_ARG_SIM_TIME} &
STATIC_TF_PID=$!
sleep 2

# Launch SLAM Toolbox
echo "[6/8] Launching SLAM toolbox..."
ros2 launch coven_core slam_toolbox_namespaced.launch.py \\
    namespace:={rover_name} \\
    use_sim_time:=true \\
    slam_params_file:={slam_config} &
SLAM_PID=$!
sleep 5  # SLAM needs time to configure and activate

# Start COVEN module
echo "[7/8] Starting COVEN module..."
ros2 run coven_core coven_module \\
    --ros-args \\
    -r {ROS_ARG_NS_PREFIX}{rover_name} \\
    -p {ROS_ARG_SIM_TIME} \\
    -p module_id:={rover_name} \\
    -p robot_namespace:={rover_name} &
COVEN_PID=$!
sleep 2

# Launch Nav2 stack
echo "[8/8] Launching Nav2 stack..."
ros2 launch coven_core coven_navigation.launch.py \\
    namespace:={rover_name} \\
    use_sim_time:=true \\
    params_file:={nav2_params_file} \\
    autostart:=true &
NAV2_PID=$!

echo "=== {rover_name} Stack Complete ==="
echo "PIDs: GZ_BRIDGE=$GZ_BRIDGE_PID, SCAN=$SCAN_PID, ODOM_TF=$ODOM_TF_PID, STATIC_TF=$STATIC_TF_PID, SLAM=$SLAM_PID, COVEN=$COVEN_PID, NAV2=$NAV2_PID"

# Wait for all background processes
wait
'''
    with open(script_path, 'w') as f:
        f.write(script_content)
    os.chmod(script_path, 0o755)
    return script_path


def generate_launch_description():
    """Generate launch description for COVEN 4-rover simulation."""

    pkg_coven = get_package_share_directory('coven_core')

    # Paths
    world_file = os.path.join(pkg_coven, 'worlds', 'coven_4rover.sdf')
    models_path = os.path.join(pkg_coven, 'models')
    nav2_params_file = os.path.join(pkg_coven, 'config', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(pkg_coven, 'config', 'coven_sim.rviz')
    rover_sdf = os.path.join(models_path, 'coven_rover', 'model.sdf')
    dock_sdf = os.path.join(models_path, 'dock.sdf')

    # Dock position
    dock_x = 0.0
    dock_y = 0.0

    # Calculate rover positions and prepare data
    rovers_data = []
    for i, rover_name in enumerate(_selected_witches):
        distance, angle = ROVER_POSITIONS[i]
        rover_x = dock_x + distance * math.cos(angle)
        rover_y = dock_y + distance * math.sin(angle)
        facing_angle = math.atan2(rover_y - dock_y, rover_x - dock_x)
        qz = math.sin(facing_angle / 2)
        qw = math.cos(facing_angle / 2)

        # Write SLAM config
        slam_config = write_slam_config(rover_name)

        # Gazebo topics
        gz_odom = f'/model/{rover_name}/odometry'
        gz_tf = f'/model/{rover_name}/tf'
        gz_scan = f'/world/coven_world/model/{rover_name}/link/base_link/sensor/lidar/scan'
        gz_cmd_vel = f'/model/{rover_name}/cmd_vel'
        gz_joint = f'/world/coven_world/model/{rover_name}/joint_state'

        # ROS topics
        ros_odom = f'/{rover_name}/odom'
        ros_cmd_vel = f'/{rover_name}/cmd_vel'
        ros_joint = f'/{rover_name}/joint_states'

        # Write rover launch script
        script_path = write_rover_launch_script(
            rover_name=rover_name,
            slam_config=slam_config,
            gz_odom=gz_odom, gz_tf=gz_tf, gz_scan=gz_scan,
            gz_cmd_vel=gz_cmd_vel, gz_joint=gz_joint,
            ros_odom=ros_odom, ros_cmd_vel=ros_cmd_vel, ros_joint=ros_joint,
            nav2_params_file=nav2_params_file,
        )

        rovers_data.append({
            'name': rover_name,
            'x': rover_x,
            'y': rover_y,
            'qz': qz,
            'qw': qw,
            'script': script_path,
        })

    # Write obstacle SDFs to /tmp
    for name, geometry_sdf, x, y, z, qx, qy, obs_qz, obs_qw in _random_obstacles:
        obstacle_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision"><geometry>{geometry_sdf}</geometry></collision>
      <visual name="visual">
        <geometry>{geometry_sdf}</geometry>
        <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.7 0.5 0.3 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>'''
        with open(f'/tmp/coven_{name}.sdf', 'w') as f:
            f.write(obstacle_sdf)

    # Write wall SDFs to /tmp
    for name, length, x, y, z, wall_qx, wall_qy, wall_qz, wall_qw in _random_interior_walls:
        wall_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision"><geometry><box><size>{length} 0.2 1.0</size></box></geometry></collision>
      <visual name="visual">
        <geometry><box><size>{length} 0.2 1.0</size></box></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>'''
        with open(f'/tmp/coven_{name}.sdf', 'w') as f:
            f.write(wall_sdf)

    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI)'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Enable verbose Gazebo output'
    )

    dock_x_arg = DeclareLaunchArgument(
        'dock_x', default_value='0.0',
        description='Dock X position'
    )

    dock_y_arg = DeclareLaunchArgument(
        'dock_y', default_value='0.0',
        description='Dock Y position'
    )

    # Environment
    set_gz_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', models_path
    )

    # ============================================================
    # STAGE 1: Gazebo Simulation
    # ============================================================
    verbose = LaunchConfiguration('verbose')
    gazebo_quiet = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '1', world_file],
        output='screen',
        condition=UnlessCondition(verbose),
        name='gazebo'
    )
    gazebo_verbose = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        condition=IfCondition(verbose),
        name='gazebo'
    )

    # ============================================================
    # STAGE 2: Clock Bridge (starts after brief delay for Gazebo)
    # The bash scripts inside terminals do actual topic-based waiting
    # ============================================================
    clock_bridge = TimerAction(
        period=5.0,  # Wait for Gazebo to initialize
        actions=[
            LogInfo(msg='[Event] Starting clock bridge'),
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
    spawn_dock = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', GZ_CREATE_SERVICE,
            '--reqtype', GZ_ENTITY_FACTORY_TYPE,
            '--reptype', GZ_BOOLEAN_TYPE,
            '--timeout', '5000',
            '--req', f'sdf_filename: "{dock_sdf}", name: "{_selected_coven}", '
                     f'pose: {{position: {{x: {dock_x}, y: {dock_y}, z: 0.05}}}}'
        ],
        output='screen',
        name='spawn_dock'
    )

    # Spawn rovers
    spawn_actions = [spawn_dock]
    for rd in rovers_data:
        spawn_actions.append(ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', GZ_CREATE_SERVICE,
                '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                '--reptype', GZ_BOOLEAN_TYPE,
                '--timeout', '5000',
                '--req', f'sdf_filename: "{rover_sdf}", name: "{rd["name"]}", '
                         f'pose: {{position: {{x: {rd["x"]}, y: {rd["y"]}, z: 0.15}}, '
                         f'orientation: {{x: 0, y: 0, z: {rd["qz"]}, w: {rd["qw"]}}}}}'
            ],
            output='screen',
            name=f'spawn_{rd["name"]}'
        ))

    # Spawn obstacles and walls
    for name, geometry_sdf, x, y, z, obs_qx, obs_qy, obs_qz, obs_qw in _random_obstacles:
        spawn_actions.append(ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', GZ_CREATE_SERVICE,
                '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                '--reptype', GZ_BOOLEAN_TYPE,
                '--timeout', '5000',
                '--req', f'sdf_filename: "/tmp/coven_{name}.sdf", name: "{name}", '
                         f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
                         f'orientation: {{x: {obs_qx}, y: {obs_qy}, z: {obs_qz}, w: {obs_qw}}}}}'
            ],
            output='screen',
            name=f'spawn_{name}'
        ))

    for name, length, x, y, z, wall_qx, wall_qy, wall_qz, wall_qw in _random_interior_walls:
        spawn_actions.append(ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', GZ_CREATE_SERVICE,
                '--reqtype', GZ_ENTITY_FACTORY_TYPE,
                '--reptype', GZ_BOOLEAN_TYPE,
                '--timeout', '5000',
                '--req', f'sdf_filename: "/tmp/coven_{name}.sdf", name: "{name}", '
                         f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
                         f'orientation: {{x: {wall_qx}, y: {wall_qy}, z: {wall_qz}, w: {wall_qw}}}}}'
            ],
            output='screen',
            name=f'spawn_{name}'
        ))

    spawn_group = TimerAction(
        period=3.0,
        actions=[LogInfo(msg='[Event] Clock ready → Spawning Gazebo models')] + spawn_actions
    )

    # ============================================================
    # STAGE 4: Rover Stacks (each in own gnome-terminal)
    # Staggered start times to avoid DDS discovery issues
    # ============================================================
    rover_terminals = []
    for i, rd in enumerate(rovers_data):
        delay = 8.0 + (i * 3.0)  # Stagger by 3 seconds for 4 rovers
        rover_terminals.append(TimerAction(
            period=delay,
            actions=[
                LogInfo(msg=f'[Event] Launching {rd["name"]} stack in terminal'),
                ExecuteProcess(
                    cmd=['gnome-terminal', '--title', f'{rd["name"]} Stack', '--', 'bash', rd['script']],
                    output='screen',
                    name=f'{rd["name"]}_terminal'
                ),
            ]
        ))

    # ============================================================
    # STAGE 5: COVEN Dock (after all rovers have started)
    # ============================================================
    dock_node = TimerAction(
        period=45.0,  # Give all 4 rovers time to initialize
        actions=[
            LogInfo(msg=f'[Event] Launching COVEN dock ({_selected_coven})'),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'coven_core', 'coven_dock',
                    '--ros-args',
                    '-p', ROS_ARG_SIM_TIME,
                    '-p', f'coven_name:={_selected_coven}',
                ],
                output='screen',
                name='coven_dock'
            ),
        ]
    )

    # ============================================================
    # STAGE 6: RViz2
    # ============================================================
    rviz_launch = TimerAction(
        period=48.0,
        actions=[
            LogInfo(msg='[Event] Launching RViz2'),
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen',
                name='rviz2'
            ),
        ]
    )

    # ============================================================
    # Build Launch Description
    # ============================================================
    ld = LaunchDescription()

    # Arguments
    ld.add_action(headless_arg)
    ld.add_action(verbose_arg)
    ld.add_action(dock_x_arg)
    ld.add_action(dock_y_arg)

    # Environment
    ld.add_action(set_gz_model_path)

    # Info
    rover_names_str = ', '.join([r['name'] for r in rovers_data])
    ld.add_action(LogInfo(msg='=== COVEN 4-Rover Simulation ==='))
    ld.add_action(LogInfo(msg=f'Rovers: {rover_names_str}'))
    ld.add_action(LogInfo(msg=f'Dock: {_selected_coven}'))

    # Launch sequence
    ld.add_action(gazebo_quiet)
    ld.add_action(gazebo_verbose)
    ld.add_action(clock_bridge)  # TimerAction waits 5s for Gazebo
    ld.add_action(spawn_group)

    # Rover terminals (staggered)
    for rt in rover_terminals:
        ld.add_action(rt)

    # Final stages
    ld.add_action(dock_node)
    ld.add_action(rviz_launch)

    return ld
