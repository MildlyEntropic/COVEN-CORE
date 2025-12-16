#!/usr/bin/env python3
"""
COVEN 2-Rover Simulation Launch (4-Window Layout)

Windows:
1. COVEN - Main sim (Gazebo, bridges, SLAM, Nav2, RViz)
2. <Coven Name> - Dock firmware (e.g., "The Norns")
3. <Rover 1 Name> - Rover 1 firmware (e.g., "Morgan Le Fay")
4. <Rover 2 Name> - Rover 2 firmware (e.g., "Circe")

Uses OnProcessExit chaining for sequential startup with 210s total (~3.5 min)

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
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

# Witch/Coven names
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

ROVER_POSITIONS = [
    (1.2, math.pi / 2),      # North
    (1.2, -math.pi / 2),     # South
]

# Gazebo constants
GZ_CREATE_SERVICE = '/world/coven_world/create'
GZ_ENTITY_FACTORY_TYPE = 'gz.msgs.EntityFactory'
GZ_BOOLEAN_TYPE = 'gz.msgs.Boolean'

# Random names at module load
_selected_witches = random.sample(WITCH_NAMES, 2)
_selected_coven = random.choice(COVEN_NAMES)


def display_name(internal_name: str) -> str:
    """Convert internal name (The_Norns, Morgan_Le_Fay) to display name (The Norns, Morgan Le Fay)."""
    return internal_name.replace('_', ' ')


def write_nav2_params(rover_name: str) -> str:
    """Write Nav2 config with namespaced frame names and return path.

    Nav2 doesn't automatically namespace frame IDs, so each rover needs
    its own params file with rover_name/base_link, rover_name/odom, etc.
    """
    config_path = f'/tmp/nav2_{rover_name}.yaml'
    content = f"""# Nav2 Parameters for {rover_name}
# Auto-generated with namespaced frame names

amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "{rover_name}/base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "{rover_name}/odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: /{rover_name}/scan

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: {rover_name}/base_link
    odom_topic: /{rover_name}/odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
    enable_groot_monitoring: false

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.03
      yaw_goal_tolerance: 0.05
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.1
      batch_size: 2000
      ax_max: 3.0
      ax_min: -3.0
      ay_max: 3.0
      az_max: 3.5
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.4
      vx_max: 0.3
      vx_min: -0.15
      vy_max: 0.0
      wz_max: 1.0
      iteration_count: 1
      prune_distance: 1.5
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      regenerate_noises: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.2
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: false
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: {rover_name}/odom
      robot_base_frame: {rover_name}/base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.15
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /{rover_name}/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: {rover_name}/base_link
      use_sim_time: true
      robot_radius: 0.15
      resolution: 0.05
      track_unknown_space: true
      rolling_window: true
      width: 30
      height: 30
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /{rover_name}/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      use_final_approach_orientation: false
      cost_travel_multiplier: 2.0

smoother_server:
  ros__parameters:
    use_sim_time: true
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    global_frame: {rover_name}/odom
    robot_base_frame: {rover_name}/base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

velocity_smoother:
  ros__parameters:
    use_sim_time: true
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.3, 0.0, 1.0]
    min_velocity: [-0.3, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: /{rover_name}/odom
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

collision_monitor:
  ros__parameters:
    use_sim_time: true
    base_frame_id: "{rover_name}/base_link"
    odom_frame_id: "{rover_name}/odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    base_shift_correction: True
    stop_pub_timeout: 2.0
    polygons: ["PolygonSlow", "PolygonStop"]
    PolygonSlow:
      type: "circle"
      radius: 0.20
      action_type: "slowdown"
      slowdown_ratio: 0.5
    PolygonStop:
      type: "circle"
      radius: 0.08
      action_type: "stop"
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/{rover_name}/scan"
      min_height: 0.05
      max_height: 0.5
      enabled: True

docking_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 50.0
    initial_perception_timeout: 5.0
    wait_charge_timeout: 5.0
    dock_approach_timeout: 30.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.05
    max_retries: 3
    base_frame: "{rover_name}/base_link"
    fixed_frame: "{rover_name}/odom"
    dock_backwards: false
    dock_prestaging_tolerance: 0.5
    dock_plugins: ["simple_charging_dock"]
    simple_charging_dock:
      plugin: "opennav_docking::SimpleChargingDock"
      docking_threshold: 0.05
      staging_x_offset: -0.5
      staging_yaw_offset: 0.0
      use_external_detection_pose: false
      use_battery_status: false
    navigator_bt_xml: ""
    controller:
      plugin: "nav2_docking::DockingController"

waypoint_follower:
  ros__parameters:
    use_sim_time: true
    loop_rate: 20
    stop_on_failure: false
    action_server_result_timeout: 900.0
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
"""
    with open(config_path, 'w') as f:
        f.write(content)
    return config_path


def write_slam_config(rover_name: str) -> str:
    """Write SLAM config and return path."""
    config_path = f'/tmp/slam_{rover_name}.yaml'
    content = f"""/**:
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
        f.write(content)
    return config_path


def write_main_script(
    pkg_coven: str,
    world_file: str,
    dock_sdf: str,
    rover_sdf: str,
    dock_name: str,
    dock_x: float,
    dock_y: float,
    rovers_data: list,
    rviz_config: str,
) -> str:
    """Write the main COVEN simulation script."""
    r1 = rovers_data[0]
    r2 = rovers_data[1]

    script = f'''#!/bin/bash
# COVEN 2-Rover Simulation - Main Script
# Proper lifecycle-based startup: Nav2 starts inactive, activated after SLAM ready
#
# Startup Chain (per ChatGPT/Nav2 best practices):
#   Phase A: Gazebo only (wait for /clock, /odom, /scan)
#   Phase B: Core robot + TF (bridges, odom_tf_broadcaster, static TF)
#   Phase C: SLAM only (wait for /map topic + map->odom TF)
#   Phase D: Nav2 with autostart=false (starts inactive)
#   Phase E: Lifecycle manager activation (after SLAM confirmed ready)
#   Phase F: RViz (last, always)

# Don't use set -e - we handle errors ourselves
source /home/avactus/ros2_ws/install/setup.bash

echo "=== COVEN 2-Rover Simulation (Lifecycle-Based Startup) ==="
echo "Dock: {display_name(dock_name)}"
echo "Rovers: {display_name(r1['name'])}, {display_name(r2['name'])}"
echo ""

# Helper function: wait for topic to exist (always returns 0, just logs warning on timeout)
wait_for_topic() {{
    local topic=$1
    local timeout=$2
    local count=0
    echo "  Waiting for topic $topic..."
    while [ $count -lt $timeout ]; do
        if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
            echo "  Topic $topic available!"
            return 0
        fi
        count=$((count + 1))
        sleep 1
    done
    echo "  WARNING: Timeout waiting for $topic (continuing anyway)"
    return 0  # Don't fail - just warn
}}

# Helper function: wait for TF to be published (always returns 0)
wait_for_tf() {{
    local parent=$1
    local child=$2
    local timeout=$3
    local count=0
    echo "  Waiting for TF $parent -> $child..."
    while [ $count -lt $timeout ]; do
        if timeout 2 ros2 run tf2_ros tf2_echo $parent $child --once 2>&1 | grep -q "At time"; then
            echo "  TF $parent -> $child available!"
            return 0
        fi
        count=$((count + 2))
        sleep 2
    done
    echo "  WARNING: Timeout waiting for TF $parent -> $child (continuing anyway)"
    return 0  # Don't fail - just warn
}}

# ============================================================
# PHASE A: Gazebo Only
# ============================================================
echo ""
echo "========== PHASE A: Gazebo =========="
echo "Starting Gazebo simulation..."
gz sim -r -v 1 {world_file} &
GAZEBO_PID=$!

# Wait for Gazebo to fully initialize (check for /clock topic)
echo "Waiting for Gazebo /clock topic..."
sleep 5  # Initial settle time
wait_for_topic "/clock" 30

# Start clock bridge immediately after /clock exists
echo "Starting clock bridge..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
sleep 3

# ============================================================
# PHASE A.1: Spawn Models
# ============================================================
echo ""
echo "Spawning dock: {dock_name}..."
gz service -s {GZ_CREATE_SERVICE} --reqtype {GZ_ENTITY_FACTORY_TYPE} --reptype {GZ_BOOLEAN_TYPE} --timeout 5000 --req 'sdf_filename: "{dock_sdf}", name: "{dock_name}", pose: {{position: {{x: {dock_x}, y: {dock_y}, z: 0.05}}}}'
sleep 2

# ============================================================
# PHASE B: Rover 1 - Core Robot + TF
# ============================================================
echo ""
echo "========== PHASE B: {r1['name']} Core + TF =========="

echo "Spawning {r1['name']}..."
gz service -s {GZ_CREATE_SERVICE} --reqtype {GZ_ENTITY_FACTORY_TYPE} --reptype {GZ_BOOLEAN_TYPE} --timeout 5000 --req 'sdf_filename: "{rover_sdf}", name: "{r1['name']}", pose: {{position: {{x: {r1['x']}, y: {r1['y']}, z: 0.15}}, orientation: {{x: 0, y: 0, z: {r1['qz']}, w: {r1['qw']}}}}}'
sleep 2

echo "Starting {r1['name']} Gazebo bridges..."
ros2 run ros_gz_bridge parameter_bridge \\
    {r1['gz_odom']}@nav_msgs/msg/Odometry[gz.msgs.Odometry \\
    {r1['gz_tf']}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
    {r1['gz_scan']}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \\
    {r1['gz_cmd_vel']}@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    {r1['gz_joint']}@sensor_msgs/msg/JointState[gz.msgs.Model \\
    --ros-args \\
    -r {r1['gz_odom']}:={r1['ros_odom']} \\
    -r {r1['gz_tf']}:=/{r1['name']}/tf \\
    -r {r1['gz_scan']}:=/{r1['name']}/scan_raw \\
    -r {r1['gz_cmd_vel']}:={r1['ros_cmd_vel']} \\
    -r {r1['gz_joint']}:={r1['ros_joint']} \\
    -p use_sim_time:=true &

echo "Starting {r1['name']} scan republisher..."
ros2 run coven_core scan_frame_republisher --ros-args \\
    -p input_topic:=/{r1['name']}/scan_raw \\
    -p output_topic:=/{r1['name']}/scan \\
    -p frame_id:={r1['name']}/base_link \\
    -p use_sim_time:=true &

echo "Starting {r1['name']} odom TF broadcaster..."
ros2 run coven_core odom_tf_broadcaster --ros-args \\
    -r __ns:=/{r1['name']} -r /tf:=/tf -r /tf_static:=/tf_static \\
    -p use_sim_time:=true \\
    -p odom_topic:={r1['ros_odom']} \\
    -p odom_frame:={r1['name']}/odom \\
    -p base_frame:={r1['name']}/base_link &

echo "Starting {r1['name']} static TF (map->odom)..."
ros2 run tf2_ros static_transform_publisher \\
    --frame-id map --child-frame-id {r1['name']}/odom \\
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \\
    --ros-args -r /tf:=/tf -r /tf_static:=/tf_static -p use_sim_time:=true &

# Wait for odom and scan to be available
wait_for_topic "/{r1['name']}/odom" 30
wait_for_topic "/{r1['name']}/scan" 30
wait_for_tf "{r1['name']}/odom" "{r1['name']}/base_link" 30

# ============================================================
# PHASE C: Rover 1 - SLAM Only
# ============================================================
echo ""
echo "========== PHASE C: {r1['name']} SLAM =========="

echo "Starting {r1['name']} SLAM..."
ros2 launch coven_core slam_toolbox_namespaced.launch.py \\
    namespace:={r1['name']} use_sim_time:=true slam_params_file:={r1['slam_config']} &

# Wait for SLAM to publish map topic AND map->odom TF
# Note: SLAM publishes to /map (global) not /rover_name/map
wait_for_topic "/map" 90
wait_for_tf "map" "{r1['name']}/odom" 90
echo "{r1['name']} SLAM ready!"

# ============================================================
# PHASE D: Rover 1 - Nav2 with autostart=false
# ============================================================
echo ""
echo "========== PHASE D: {r1['name']} Nav2 (inactive) =========="

echo "Starting {r1['name']} Nav2 (autostart=false)..."
ros2 launch coven_core coven_navigation.launch.py \\
    namespace:={r1['name']} use_sim_time:=true \\
    params_file:={r1['nav2_config']} autostart:=false &

# Wait for lifecycle manager to exist
echo "Waiting for {r1['name']} lifecycle manager..."
sleep 5
while ! ros2 node list 2>/dev/null | grep -q "/{r1['name']}/lifecycle_manager_navigation"; do
    sleep 2
done
echo "{r1['name']} lifecycle manager up!"

# ============================================================
# PHASE E: Rover 1 - Activate Nav2 lifecycle
# ============================================================
echo ""
echo "========== PHASE E: {r1['name']} Nav2 Activation =========="

# The lifecycle manager handles transitioning all nodes when we call startup
echo "Activating {r1['name']} Nav2 stack via lifecycle manager..."
ros2 service call /{r1['name']}/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{{command: 0}}"
# command: 0 = startup (configure + activate all nodes)
# Note: removed '&' - wait for activation to complete before proceeding

# Wait for bt_navigator action server to be ready (proves Nav2 is truly active)
# The action server only accepts goals when the node is in the 'active' lifecycle state
echo "Waiting for {r1['name']} bt_navigator action server to be ready..."
while ! ros2 action list 2>/dev/null | grep -q "/{r1['name']}/navigate_to_pose"; do
    sleep 2
done
echo "{r1['name']} Nav2 ACTIVE and accepting goals!"

# ============================================================
# PHASE B2: Rover 2 - Core Robot + TF
# ============================================================
echo ""
echo "========== PHASE B2: {r2['name']} Core + TF =========="

echo "Spawning {r2['name']}..."
gz service -s {GZ_CREATE_SERVICE} --reqtype {GZ_ENTITY_FACTORY_TYPE} --reptype {GZ_BOOLEAN_TYPE} --timeout 5000 --req 'sdf_filename: "{rover_sdf}", name: "{r2['name']}", pose: {{position: {{x: {r2['x']}, y: {r2['y']}, z: 0.15}}, orientation: {{x: 0, y: 0, z: {r2['qz']}, w: {r2['qw']}}}}}'
sleep 2

echo "Starting {r2['name']} Gazebo bridges..."
ros2 run ros_gz_bridge parameter_bridge \\
    {r2['gz_odom']}@nav_msgs/msg/Odometry[gz.msgs.Odometry \\
    {r2['gz_tf']}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
    {r2['gz_scan']}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \\
    {r2['gz_cmd_vel']}@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    {r2['gz_joint']}@sensor_msgs/msg/JointState[gz.msgs.Model \\
    --ros-args \\
    -r {r2['gz_odom']}:={r2['ros_odom']} \\
    -r {r2['gz_tf']}:=/{r2['name']}/tf \\
    -r {r2['gz_scan']}:=/{r2['name']}/scan_raw \\
    -r {r2['gz_cmd_vel']}:={r2['ros_cmd_vel']} \\
    -r {r2['gz_joint']}:={r2['ros_joint']} \\
    -p use_sim_time:=true &

echo "Starting {r2['name']} scan republisher..."
ros2 run coven_core scan_frame_republisher --ros-args \\
    -p input_topic:=/{r2['name']}/scan_raw \\
    -p output_topic:=/{r2['name']}/scan \\
    -p frame_id:={r2['name']}/base_link \\
    -p use_sim_time:=true &

echo "Starting {r2['name']} odom TF broadcaster..."
ros2 run coven_core odom_tf_broadcaster --ros-args \\
    -r __ns:=/{r2['name']} -r /tf:=/tf -r /tf_static:=/tf_static \\
    -p use_sim_time:=true \\
    -p odom_topic:={r2['ros_odom']} \\
    -p odom_frame:={r2['name']}/odom \\
    -p base_frame:={r2['name']}/base_link &

echo "Starting {r2['name']} static TF (map->odom)..."
ros2 run tf2_ros static_transform_publisher \\
    --frame-id map --child-frame-id {r2['name']}/odom \\
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \\
    --ros-args -r /tf:=/tf -r /tf_static:=/tf_static -p use_sim_time:=true &

# Wait for odom and scan to be available
wait_for_topic "/{r2['name']}/odom" 30
wait_for_topic "/{r2['name']}/scan" 30
wait_for_tf "{r2['name']}/odom" "{r2['name']}/base_link" 30

# ============================================================
# PHASE C2: Rover 2 - SLAM Only
# ============================================================
echo ""
echo "========== PHASE C2: {r2['name']} SLAM =========="

echo "Starting {r2['name']} SLAM..."
ros2 launch coven_core slam_toolbox_namespaced.launch.py \\
    namespace:={r2['name']} use_sim_time:=true slam_params_file:={r2['slam_config']} &

# Wait for SLAM to publish map topic AND map->odom TF
# Note: SLAM publishes to /map (global) - already exists from rover 1
# Just wait for the TF from this rover's SLAM
wait_for_tf "map" "{r2['name']}/odom" 90
echo "{r2['name']} SLAM ready!"

# ============================================================
# PHASE D2: Rover 2 - Nav2 with autostart=false
# ============================================================
echo ""
echo "========== PHASE D2: {r2['name']} Nav2 (inactive) =========="

echo "Starting {r2['name']} Nav2 (autostart=false)..."
ros2 launch coven_core coven_navigation.launch.py \\
    namespace:={r2['name']} use_sim_time:=true \\
    params_file:={r2['nav2_config']} autostart:=false &

# Wait for lifecycle manager to exist
echo "Waiting for {r2['name']} lifecycle manager..."
sleep 5
while ! ros2 node list 2>/dev/null | grep -q "/{r2['name']}/lifecycle_manager_navigation"; do
    sleep 2
done
echo "{r2['name']} lifecycle manager up!"

# ============================================================
# PHASE E2: Rover 2 - Activate Nav2 lifecycle
# ============================================================
echo ""
echo "========== PHASE E2: {r2['name']} Nav2 Activation =========="

# Double-check TF is still available before activation (odom_tf_broadcaster may have race condition)
echo "Verifying {r2['name']} TF chain is stable..."
wait_for_tf "{r2['name']}/odom" "{r2['name']}/base_link" 30
echo "{r2['name']} TF chain confirmed!"

echo "Activating {r2['name']} Nav2 stack via lifecycle manager..."
ros2 service call /{r2['name']}/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{{command: 0}}"
# Note: removed '&' - wait for activation to complete before proceeding

# Wait for bt_navigator action server to be ready (proves Nav2 is truly active)
echo "Waiting for {r2['name']} bt_navigator action server to be ready..."
while ! ros2 action list 2>/dev/null | grep -q "/{r2['name']}/navigate_to_pose"; do
    sleep 2
done
echo "{r2['name']} Nav2 ACTIVE and accepting goals!"

# ============================================================
# PHASE F: RViz (last, always)
# ============================================================
echo ""
echo "========== PHASE F: RViz =========="
sleep 5
echo "Launching RViz2..."
rviz2 -d {rviz_config} &

echo ""
echo "============================================"
echo "=== COVEN Simulation Ready ==="
echo "Both rovers have active Nav2 stacks!"
echo "============================================"
wait
'''

    script_path = '/tmp/coven_main.sh'
    with open(script_path, 'w') as f:
        f.write(script)
    os.chmod(script_path, 0o755)
    return script_path


def write_dock_script(dock_name: str, r1_name: str, r2_name: str) -> str:
    """Write the dock firmware script."""
    script = f'''#!/bin/bash
# {display_name(dock_name)} - Dock Firmware
source /home/avactus/ros2_ws/install/setup.bash

echo "=== {display_name(dock_name)} ==="
echo "Waiting for both rovers to be ready..."

# Wait for BOTH rovers' nav2 stacks to be up (bt_navigator proves Nav2 is running)
echo "Waiting for {display_name(r1_name)} Nav2..."
while ! ros2 node list 2>/dev/null | grep -q "/{r1_name}/bt_navigator"; do
    sleep 3
done
echo "{display_name(r1_name)} Nav2 ready!"

echo "Waiting for {display_name(r2_name)} Nav2..."
while ! ros2 node list 2>/dev/null | grep -q "/{r2_name}/bt_navigator"; do
    sleep 3
done
echo "{display_name(r2_name)} Nav2 ready!"

echo "Both rovers ready, starting dock firmware..."
exec ros2 run coven_core coven_dock --ros-args \\
    -p use_sim_time:=true \\
    -p coven_name:={dock_name}
'''

    script_path = '/tmp/coven_dock.sh'
    with open(script_path, 'w') as f:
        f.write(script)
    os.chmod(script_path, 0o755)
    return script_path


def write_rover_script(rover_name: str) -> str:
    """Write a rover firmware script."""
    display = display_name(rover_name)
    script = f'''#!/bin/bash
# {display} - Rover Firmware
source /home/avactus/ros2_ws/install/setup.bash

echo "=== {display} ==="
echo "Waiting for rover stack to be ready..."

# Wait for SLAM to be publishing (SLAM publishes to /map, not namespaced)
echo "Waiting for /map topic..."
while ! ros2 topic list 2>/dev/null | grep -q "^/map$"; do
    sleep 3
done
echo "SLAM active!"

# Wait for Nav2 action server to be ready (proves Nav2 is truly active)
echo "Waiting for Nav2 action server..."
while ! ros2 action list 2>/dev/null | grep -q "/{rover_name}/navigate_to_pose"; do
    sleep 3
done
echo "Nav2 ACTIVE and accepting goals!"

echo "Starting rover module firmware..."
exec ros2 run coven_core coven_module --ros-args \\
    -r __ns:=/{rover_name} \\
    -p use_sim_time:=true \\
    -p module_id:={rover_name} \\
    -p robot_namespace:={rover_name}
'''

    script_path = f'/tmp/coven_rover_{rover_name}.sh'
    with open(script_path, 'w') as f:
        f.write(script)
    os.chmod(script_path, 0o755)
    return script_path


def generate_launch_description():
    pkg_coven = get_package_share_directory('coven_core')

    # Paths
    world_file = os.path.join(pkg_coven, 'worlds', 'coven_4rover.sdf')
    models_path = os.path.join(pkg_coven, 'models')
    rviz_config = os.path.join(pkg_coven, 'config', 'coven_sim.rviz')
    rover_sdf = os.path.join(models_path, 'coven_rover', 'model.sdf')
    dock_sdf = os.path.join(models_path, 'dock.sdf')

    dock_x, dock_y = 0.0, 0.0

    # Calculate rover data
    rovers_data = []
    for i, rover_name in enumerate(_selected_witches):
        distance, angle = ROVER_POSITIONS[i]
        rover_x = dock_x + distance * math.cos(angle)
        rover_y = dock_y + distance * math.sin(angle)
        facing_angle = math.atan2(rover_y - dock_y, rover_x - dock_x)
        qz = math.sin(facing_angle / 2)
        qw = math.cos(facing_angle / 2)

        gz_odom = f'/model/{rover_name}/odometry'
        gz_tf = f'/model/{rover_name}/tf'
        gz_scan = f'/world/coven_world/model/{rover_name}/link/base_link/sensor/lidar/scan'
        gz_cmd_vel = f'/model/{rover_name}/cmd_vel'
        gz_joint = f'/world/coven_world/model/{rover_name}/joint_state'

        slam_config = write_slam_config(rover_name)
        nav2_config = write_nav2_params(rover_name)

        rovers_data.append({
            'name': rover_name,
            'x': rover_x, 'y': rover_y, 'qz': qz, 'qw': qw,
            'gz_odom': gz_odom, 'gz_tf': gz_tf, 'gz_scan': gz_scan,
            'gz_cmd_vel': gz_cmd_vel, 'gz_joint': gz_joint,
            'ros_odom': f'/{rover_name}/odom',
            'ros_cmd_vel': f'/{rover_name}/cmd_vel',
            'ros_joint': f'/{rover_name}/joint_states',
            'slam_config': slam_config,
            'nav2_config': nav2_config,
        })

    # Write scripts
    main_script = write_main_script(
        pkg_coven, world_file, dock_sdf, rover_sdf,
        _selected_coven, dock_x, dock_y, rovers_data,
        rviz_config
    )
    dock_script = write_dock_script(_selected_coven, rovers_data[0]['name'], rovers_data[1]['name'])
    # Rovers wait for their own topics to exist (no fixed delays)
    r1_script = write_rover_script(rovers_data[0]['name'])
    r2_script = write_rover_script(rovers_data[1]['name'])

    # Display names for window titles
    dock_display = display_name(_selected_coven)
    r1_display = display_name(rovers_data[0]['name'])
    r2_display = display_name(rovers_data[1]['name'])

    # Launch description
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('verbose', default_value='false'))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path))

    ld.add_action(LogInfo(msg='=== COVEN 2-Rover Simulation (4-Window Layout) ==='))
    ld.add_action(LogInfo(msg=f'Dock: {dock_display}'))
    ld.add_action(LogInfo(msg=f'Rovers: {r1_display}, {r2_display}'))

    # Main sim runs in the current terminal (no new window)
    coven_main = ExecuteProcess(
        cmd=['bash', main_script],
        output='screen',
        name='coven_main'
    )

    # Window 2: Dock firmware
    dock_terminal = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--title', dock_display,
            '--', 'bash', dock_script
        ],
        output='screen',
        name='dock_terminal'
    )

    # Window 3: Rover 1 firmware
    r1_terminal = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--title', r1_display,
            '--', 'bash', r1_script
        ],
        output='screen',
        name='r1_terminal'
    )

    # Window 4: Rover 2 firmware
    r2_terminal = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--title', r2_display,
            '--', 'bash', r2_script
        ],
        output='screen',
        name='r2_terminal'
    )

    # Launch all terminals
    ld.add_action(coven_main)
    ld.add_action(dock_terminal)
    ld.add_action(r1_terminal)
    ld.add_action(r2_terminal)

    return ld
