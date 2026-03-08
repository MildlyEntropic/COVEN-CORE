#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
offline_slam_processor.py - Dock-Side Offline SLAM Processor

Processes recorded sensor data from rovers and builds/updates
the map using SLAM Toolbox in offline mode.

When a rover docks and transfers its recorded sensor data, this node:
1. Reads the recorded mission data (JSON file with LiDAR + odom frames)
2. Replays frames to SLAM Toolbox at configurable speed
3. Updates the global map with new observations
4. Saves updated map

This runs on the dock - rovers never see or build the map.

Author: Alexander Shultis
Date: December 2025
"""

import os
import json
import shutil
import time
from typing import Optional
from dataclasses import dataclass
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import String

from tf2_ros import TransformBroadcaster
import math


@dataclass
class QueuedMission:
    """A mission waiting to be processed."""
    filename: str
    mission_id: str
    module_id: str
    frame_count: int


class OfflineSLAMProcessor(Node):
    """
    Processes recorded sensor data and feeds it to SLAM Toolbox.

    When a rover uploads its sensor batch, this node reads the recorded data
    and replays it (at accelerated speed) to build the map.
    """

    def __init__(self):
        super().__init__('offline_slam_processor')

        # Parameters
        self.declare_parameter('data_dir', '/tmp/coven_sensor_data')
        self.declare_parameter('playback_speed', 10.0)  # 10x faster than real-time
        self.declare_parameter('scan_topic', '/offline_scan')
        self.declare_parameter('odom_topic', '/offline_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('slam_output_dir', '')  # Where to save maps

        self.data_dir = self.get_parameter('data_dir').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.slam_output_dir = self.get_parameter('slam_output_dir').value

        # Track missions for incremental map naming
        self.missions_processed = 0

        # State
        self.processing = False
        self.current_mission_id: Optional[str] = None
        self.current_module_id: Optional[str] = None
        self.current_slam_dir: Optional[str] = None  # Derived from data file path
        self.session_start_time: Optional[float] = None  # For elapsed time calculation

        # Queue for missions that arrive while processing
        self.mission_queue: deque[QueuedMission] = deque()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to sensor batch notifications from rovers
        self.sensor_batch_sub = self.create_subscription(
            String, '/coven/sensor_batch', self._sensor_batch_callback, reliable_qos
        )

        # Publishers for SLAM Toolbox input
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, sensor_qos)

        # TF broadcaster for odom->base_link during replay
        self.tf_broadcaster = TransformBroadcaster(self)

        # Status publisher
        self.status_pub = self.create_publisher(
            String, '/coven/slam_processor_status', reliable_qos
        )

        self.get_logger().info(
            f'[OfflineSLAM] Processor ready, watching {self.data_dir}'
        )

    def _sensor_batch_callback(self, msg: String):
        """Handle sensor batch notification from rover."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get('type') != 'transfer_complete':
            return

        filename = data.get('filename')
        mission_id = data.get('mission_id')
        module_id = data.get('module_id', 'unknown')
        frame_count = data.get('frame_count', 0)

        if not filename or not os.path.exists(filename):
            self.get_logger().error(f'[OfflineSLAM] Data file not found: {filename}')
            return

        # Create queued mission
        queued = QueuedMission(
            filename=filename,
            mission_id=mission_id,
            module_id=module_id,
            frame_count=frame_count
        )

        if self.processing:
            # Queue for later processing
            self.mission_queue.append(queued)
            self.get_logger().info(
                f'[OfflineSLAM] Queued mission {mission_id} from {module_id} '
                f'({frame_count} frames) - Queue size: {len(self.mission_queue)}'
            )
        else:
            # Process immediately
            self.get_logger().info(
                f'[OfflineSLAM] Processing mission {mission_id} from {module_id} ({frame_count} frames)'
            )
            self._process_mission_data(queued)

    def _process_mission_data(self, mission: QueuedMission):
        """Kick off replay in a background thread to avoid blocking the ROS2 executor."""
        self.processing = True
        self.current_mission_id = mission.mission_id
        self.current_module_id = mission.module_id

        import threading
        thread = threading.Thread(
            target=self._replay_mission_thread,
            args=(mission,),
            daemon=True,
        )
        thread.start()

    def _replay_mission_thread(self, mission: QueuedMission):
        """Background thread: load and replay recorded sensor data to SLAM."""
        # Derive SLAM directory from data file path
        # Path format: .../Data/{session}/{coven}/{rover}/Scan[...].json
        # We want: .../Data/{session}/{coven}/SLAM/
        data_dir = os.path.dirname(mission.filename)  # .../rover/
        coven_dir = os.path.dirname(data_dir)          # .../coven/
        self.current_slam_dir = os.path.join(coven_dir, 'SLAM')
        os.makedirs(self.current_slam_dir, exist_ok=True)

        # Track session start time (first mission sets it)
        if self.session_start_time is None:
            self.session_start_time = time.time()

        self._publish_status('processing', mission.mission_id)

        try:
            with open(mission.filename, 'r') as f:
                mission_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'[OfflineSLAM] Failed to load {mission.filename}: {e}')
            self._finish_processing()
            return

        frames = mission_data.get('frames', [])
        if not frames:
            self.get_logger().warning('[OfflineSLAM] No frames in mission data')
            self._finish_processing()
            return

        # Get rover's initial position (world frame) to offset odometry
        initial_x = mission_data.get('initial_x', 0.0)
        initial_y = mission_data.get('initial_y', 0.0)
        initial_theta = mission_data.get('initial_theta', 0.0)

        self.get_logger().info(
            f'[OfflineSLAM] Replaying {len(frames)} frames from {mission.module_id} '
            f'(start: {initial_x:.2f}, {initial_y:.2f}, {math.degrees(initial_theta):.1f}°)'
        )

        # Calculate replay timing
        frame_delay = 1.0 / (10.0 * self.playback_speed)  # Assuming 10Hz recording

        for i, frame in enumerate(frames):
            if not rclpy.ok():
                break

            # Publish sensor data with initial position offset
            self._publish_frame(frame, initial_x, initial_y, initial_theta)

            # Progress update every 100 frames
            if i % 100 == 0:
                progress = (i / len(frames)) * 100
                self.get_logger().info(f'[OfflineSLAM] [{mission.module_id}] Progress: {progress:.1f}%')

            # Sleep for replay timing
            time.sleep(frame_delay)

        self.get_logger().info(
            f'[OfflineSLAM] Mission {mission.mission_id} from {mission.module_id} complete'
        )

        # Save map with versioning: archive current MAP -> MAP{elapsed}.{rover}, then save new MAP
        self.missions_processed += 1
        self._save_map_versioned(mission.module_id)

        self._publish_status('complete', mission.mission_id)

        self._finish_processing()

    def _finish_processing(self):
        """Finish current processing and check queue for next mission."""
        self.processing = False
        self.current_mission_id = None
        self.current_module_id = None

        # Check if there are queued missions
        if self.mission_queue:
            next_mission = self.mission_queue.popleft()
            remaining = len(self.mission_queue)
            self.get_logger().info(
                f'[OfflineSLAM] Processing queued mission {next_mission.mission_id} '
                f'from {next_mission.module_id} ({next_mission.frame_count} frames) - '
                f'{remaining} remaining in queue'
            )
            self._process_mission_data(next_mission)

    def _publish_frame(self, frame: dict, init_x: float = 0.0, init_y: float = 0.0, init_theta: float = 0.0):
        """Publish a single recorded frame to SLAM topics.

        The frame's odom values are in the rover's local frame (starting at 0,0,0).
        We need to transform them to the world frame using the rover's initial position.
        """
        now = self.get_clock().now()

        # Get frame's local odometry
        local_x = frame.get('odom_x', 0.0)
        local_y = frame.get('odom_y', 0.0)
        local_theta = frame.get('odom_theta', 0.0)

        # Transform to world frame:
        # Rotate local position by initial theta, then add initial position
        cos_t = math.cos(init_theta)
        sin_t = math.sin(init_theta)
        world_x = init_x + (local_x * cos_t - local_y * sin_t)
        world_y = init_y + (local_x * sin_t + local_y * cos_t)
        world_theta = init_theta + local_theta

        # Publish LaserScan
        scan_msg = LaserScan()
        scan_msg.header.stamp = now.to_msg()
        scan_msg.header.frame_id = self.base_frame
        scan_msg.angle_min = frame.get('scan_angle_min', -3.14159)
        scan_msg.angle_max = frame.get('scan_angle_max', 3.14159)
        scan_msg.angle_increment = frame.get('scan_angle_increment', 0.01745)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0
        scan_msg.ranges = [float(r) for r in frame.get('scan_ranges', [])]
        scan_msg.intensities = []

        self.scan_pub.publish(scan_msg)

        # Publish Odometry in world frame
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = world_x
        odom_msg.pose.pose.position.y = world_y
        odom_msg.pose.pose.position.z = 0.0

        # Convert world theta to quaternion
        odom_msg.pose.pose.orientation = self._yaw_to_quaternion(world_theta)

        # Velocities stay in local frame (robot's perspective)
        odom_msg.twist.twist.linear.x = frame.get('odom_vx', 0.0)
        odom_msg.twist.twist.linear.y = frame.get('odom_vy', 0.0)
        odom_msg.twist.twist.angular.z = frame.get('odom_vtheta', 0.0)

        self.odom_pub.publish(odom_msg)

        # Broadcast TF with world frame position
        self._broadcast_tf_world(world_x, world_y, world_theta, now)

    def _broadcast_tf_world(self, world_x: float, world_y: float, world_theta: float, stamp):
        """Broadcast odom->base_link transform in world frame."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = world_x
        t.transform.translation.y = world_y
        t.transform.translation.z = 0.0

        q = self._yaw_to_quaternion(world_theta)
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _save_map_versioned(self, module_id: str):
        """
        Save current map with versioning scheme.

        Format:
        - {coven}/SLAM/Map.pgm - always the latest map
        - {coven}/SLAM/Map.{D.HHMM.SS}.{rover}.pgm - archived versions

        Before saving new map, renames existing Map.pgm to Map.{elapsed}.{rover}.pgm
        """
        if not self.current_slam_dir:
            self.get_logger().warning('[OfflineSLAM] No SLAM directory set, skipping map save')
            return

        import subprocess

        # Calculate elapsed time since session start (D.HHMM.SS format, 24h clock)
        if self.session_start_time:
            elapsed_secs = time.time() - self.session_start_time
            elapsed_days = int(elapsed_secs // 86400)
            remaining = int(elapsed_secs % 86400)
            elapsed_hours = remaining // 3600
            elapsed_mins = (remaining % 3600) // 60
            elapsed_sec = remaining % 60
            elapsed_str = f"{elapsed_days}.{elapsed_hours:02d}{elapsed_mins:02d}.{elapsed_sec:02d}"
        else:
            elapsed_str = "0.0000.00"

        current_map_path = os.path.join(self.current_slam_dir, 'Map')
        archive_map_path = os.path.join(self.current_slam_dir, f'Map.{elapsed_str}.{module_id}')

        # Archive existing map if it exists (rename Map.pgm -> Map.{elapsed}.{rover}.pgm)
        if os.path.exists(f'{current_map_path}.pgm'):
            try:
                shutil.move(f'{current_map_path}.pgm', f'{archive_map_path}.pgm')
                shutil.move(f'{current_map_path}.yaml', f'{archive_map_path}.yaml')
                self.get_logger().info(f'[OfflineSLAM] Archived previous map to Map.{elapsed_str}.{module_id}')
            except Exception as e:
                self.get_logger().warning(f'[OfflineSLAM] Failed to archive previous map: {e}')

        # Wait for SLAM to finish processing
        self.get_logger().info(f'[OfflineSLAM] Waiting for SLAM to process, then saving to {current_map_path}...')
        time.sleep(2.0)

        try:
            # Use map_saver_cli from nav2_map_server
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', current_map_path,
                 '-t', 'map',
                 '--ros-args', '-p', 'use_sim_time:=false'],
                capture_output=True,
                text=True,
                timeout=30
            )
            if result.returncode == 0:
                self.get_logger().info(f'[OfflineSLAM] Map saved: {current_map_path}.pgm/.yaml')
            else:
                self.get_logger().warning(f'[OfflineSLAM] Map save may have failed: {result.stderr[:200] if result.stderr else "no error output"}')
        except subprocess.TimeoutExpired:
            self.get_logger().warning('[OfflineSLAM] Map save timed out')
        except Exception as e:
            self.get_logger().error(f'[OfflineSLAM] Map save error: {e}')

    def _publish_status(self, status: str, mission_id: str):
        """Publish processor status."""
        msg = String()
        msg.data = json.dumps({
            'processor': 'offline_slam',
            'status': status,
            'mission_id': mission_id
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OfflineSLAMProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
