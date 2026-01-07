#!/usr/bin/env python3
"""
data_mule_module.py - Data Mule Rover Node

A lightweight rover that:
1. Records raw sensor data (LiDAR + odometry) during exploration
2. Uses simple reactive navigation (no SLAM, no map)
3. Returns to dock and transfers recorded data physically
4. No wireless data streaming - all data transferred at dock

This makes the rover "dumber, lighter, and more modular" - it's just
a mobile sensor platform that collects data for the dock to process.

Author: Alexander Shultis
Date: December 2025
"""

import os
import time
import json
import math
from dataclasses import dataclass, field, asdict
from typing import List, Optional
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from coven_core import common


class MuleState(Enum):
    """Mule operational states."""
    IDLE = "idle"                    # At dock, waiting for mission
    RECORDING = "recording"          # Out exploring, recording data
    RETURNING = "returning"          # Heading back to dock
    TRANSFERRING = "transferring"    # At dock, transferring data
    COMPLETE = "complete"            # Transfer done


@dataclass
class SensorFrame:
    """A single timestamped sensor reading."""
    timestamp: float
    scan_ranges: List[float]
    scan_angle_min: float
    scan_angle_max: float
    scan_angle_increment: float
    odom_x: float
    odom_y: float
    odom_theta: float
    odom_vx: float
    odom_vy: float
    odom_vtheta: float


@dataclass
class RecordedMission:
    """Complete recorded data from a mission."""
    mission_id: str
    module_id: str
    start_time: float
    end_time: float = 0.0
    # Rover's initial position at mission start (world frame, from first odom reading)
    # This is needed so offline SLAM can place scans correctly on the map
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_theta: float = 0.0
    frames: List[SensorFrame] = field(default_factory=list)
    waypoints_visited: List[str] = field(default_factory=list)

    def to_json(self) -> str:
        """Serialize to JSON for transfer."""
        data = asdict(self)
        return json.dumps(data)

    @classmethod
    def from_json(cls, json_str: str) -> 'RecordedMission':
        """Deserialize from JSON."""
        data = json.loads(json_str)
        frames = [SensorFrame(**f) for f in data.pop('frames', [])]
        return cls(**data, frames=frames)


class DataMuleModule(Node):
    """
    Data Mule Rover - collects raw sensor data without onboard SLAM.

    Navigation is purely reactive:
    - Obstacle avoidance using LiDAR
    - Dead reckoning using odometry
    - Simple waypoint following (heading toward target)
    - Return-to-dock using stored initial position
    """

    def __init__(self):
        super().__init__('data_mule_module')

        # Parameters
        self.declare_parameter('module_id', 'mule1')
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('record_rate_hz', 10.0)
        self.declare_parameter('max_mission_frames', 10000)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('dock_threshold', 0.3)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('session_timestamp', '')  # YYYYMMDD.HHMM.SS
        self.declare_parameter('coven_name', '')  # Coven (dock) name
        # Spawn position - rover teleports back here after mission (sim only)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)  # In radians

        self.module_id = self.get_parameter('module_id').value
        self.namespace = self.get_parameter('robot_namespace').value
        self.record_rate = self.get_parameter('record_rate_hz').value
        self.max_frames = self.get_parameter('max_mission_frames').value
        self.obstacle_thresh = self.get_parameter('obstacle_threshold').value
        self.dock_thresh = self.get_parameter('dock_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Build data directory path: ~/Desktop/COVEN/Data/{session}/{coven}/{witch}/
        session_timestamp = self.get_parameter('session_timestamp').value
        coven_name = self.get_parameter('coven_name').value

        from datetime import datetime
        home = os.path.expanduser('~')
        base_dir = os.path.join(home, 'Desktop', 'COVEN', 'Data')

        # Use provided session_timestamp or generate new one
        if not session_timestamp:
            session_timestamp = datetime.now().strftime('%Y%m%d.%H%M.%S')

        # Use provided coven_name or default
        if not coven_name:
            coven_name = 'Unknown_Coven'

        self.data_dir = os.path.join(base_dir, session_timestamp, coven_name, self.module_id)

        os.makedirs(self.data_dir, exist_ok=True)
        self.get_logger().info(f'[DataMule] Data directory: {self.data_dir}')

        # State
        self.state = MuleState.IDLE
        self.current_mission: Optional[RecordedMission] = None
        self.dock_position = (0.0, 0.0)
        self.current_pose = (0.0, 0.0, 0.0)
        self.current_vel = (0.0, 0.0, 0.0)
        self.latest_scan: Optional[LaserScan] = None
        self.target_waypoint: Optional[tuple] = None
        self.waypoint_queue: List[tuple] = []
        self.last_record_time = 0.0

        # Spawn position for teleporting back after mission (sim only)
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_yaw = self.get_parameter('spawn_yaw').value

        # Heartbeat state
        self.hb_seq = 0
        self.hb_timer = None
        self.hb_period = 0.8

        # QoS profiles
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

        # Subscribers
        scan_topic = f'/{self.namespace}/scan' if self.namespace else '/scan'
        odom_topic = f'/{self.namespace}/odom' if self.namespace else '/odom'

        self.get_logger().info(f'[DataMule] Subscribing to scan: {scan_topic}')
        self.get_logger().info(f'[DataMule] Subscribing to odom: {odom_topic}')

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, sensor_qos
        )

        # Mission commands from dock
        self.mission_sub = self.create_subscription(
            String, '/coven/mule_mission', self._mission_callback, reliable_qos
        )

        # Publishers
        cmd_topic = f'/{self.namespace}/cmd_vel' if self.namespace else '/cmd_vel'
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, reliable_qos)

        # Status and data transfer
        self.status_pub = self.create_publisher(String, '/coven/mule_status', reliable_qos)
        self.data_pub = self.create_publisher(String, '/coven/mule_data', reliable_qos)

        # Heartbeat publisher
        self.hb_pub = self.create_publisher(String, '/coven/heartbeat', 10)

        # Timers
        self.control_timer = self.create_timer(0.05, self._control_loop)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(f'[DataMule] {self.module_id} initialized in IDLE state')
        self._start_heartbeat()

    def _scan_callback(self, msg: LaserScan):
        """Store latest scan."""
        self.latest_scan = msg
        if not hasattr(self, '_scan_received'):
            self._scan_received = True
            self.get_logger().info(f'[DataMule] First scan received ({len(msg.ranges)} ranges)')

    def _odom_callback(self, msg: Odometry):
        """Update current pose from odometry."""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (pos.x, pos.y, theta)

        vel = msg.twist.twist
        self.current_vel = (vel.linear.x, vel.linear.y, vel.angular.z)

        if not hasattr(self, '_odom_received'):
            self._odom_received = True
            self.get_logger().info(f'[DataMule] First odom received (pos: {pos.x:.2f}, {pos.y:.2f})')

    def _mission_callback(self, msg: String):
        """Handle mission commands."""
        cmd = msg.data.strip()
        self.get_logger().info(f'[DataMule] Received command: {cmd}')

        if cmd.startswith('START:'):
            if self.state != MuleState.IDLE:
                self.get_logger().warn('[DataMule] Cannot start - not idle')
                return

            parts = cmd.split(':')
            mission_id = parts[1] if len(parts) > 1 else f'mission_{int(time.time())}'

            self.waypoint_queue = []
            if len(parts) > 2 and parts[2]:
                for wp_str in parts[2].split(';'):
                    if ',' in wp_str:
                        x, y = wp_str.split(',')
                        self.waypoint_queue.append((float(x), float(y)))

            self._start_mission(mission_id)

        elif cmd == 'ABORT':
            self.get_logger().warn('[DataMule] Mission aborted')
            self._stop_and_idle()

        elif cmd == 'RETURN':
            if self.state == MuleState.RECORDING:
                self.get_logger().info('[DataMule] Returning to dock')
                self.state = MuleState.RETURNING

    def _start_mission(self, mission_id: str):
        """Initialize a new mission."""
        self.dock_position = self.current_pose[:2]

        # Store initial position - this is where the rover starts in the world
        # The odometry values are relative to this position
        x, y, theta = self.current_pose

        self.current_mission = RecordedMission(
            mission_id=mission_id,
            module_id=self.module_id,
            start_time=time.time(),
            initial_x=x,
            initial_y=y,
            initial_theta=theta
        )

        self.get_logger().info(
            f'[DataMule] Initial position: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)'
        )

        if self.waypoint_queue:
            self.target_waypoint = self.waypoint_queue.pop(0)
        else:
            x, y, theta = self.current_pose
            self.target_waypoint = (
                x + 3.0 * math.cos(theta),
                y + 3.0 * math.sin(theta)
            )

        self.state = MuleState.RECORDING
        self._stop_heartbeat()
        self.get_logger().info(f'[DataMule] Mission {mission_id} started, heading to {self.target_waypoint}')

    def _stop_and_idle(self):
        """Stop motors and return to idle."""
        self._stop()
        self.state = MuleState.IDLE
        self.current_mission = None
        self.target_waypoint = None
        self.waypoint_queue = []
        self._start_heartbeat()

    def _stop(self):
        """Stop all motors."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _control_loop(self):
        """Main control loop."""
        if self.state == MuleState.IDLE:
            self._stop()
            return

        if self.state == MuleState.RECORDING:
            self._record_frame()
            self._navigate_to_waypoint()

            if self.current_mission and len(self.current_mission.frames) >= self.max_frames:
                self.get_logger().warn('[DataMule] Frame limit reached, returning')
                self.state = MuleState.RETURNING

        elif self.state == MuleState.RETURNING:
            self._record_frame()
            self._navigate_to_dock()

            x, y, _ = self.current_pose
            dx = x - self.dock_position[0]
            dy = y - self.dock_position[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.dock_thresh:
                self.get_logger().info('[DataMule] Arrived at dock, transferring data')
                self._stop()
                self.state = MuleState.TRANSFERRING
                self._transfer_data()

        elif self.state == MuleState.TRANSFERRING:
            self._stop()

        elif self.state == MuleState.COMPLETE:
            self._stop()

    def _record_frame(self):
        """Record current sensor state."""
        now = time.time()
        if now - self.last_record_time < (1.0 / self.record_rate):
            return

        if not self.latest_scan or not self.current_mission:
            if not hasattr(self, '_record_debug_logged'):
                self._record_debug_logged = True
                self.get_logger().warn(
                    f'[DataMule] Cannot record: scan={self.latest_scan is not None}, '
                    f'mission={self.current_mission is not None}'
                )
            return

        frame = SensorFrame(
            timestamp=now,
            scan_ranges=list(self.latest_scan.ranges),
            scan_angle_min=self.latest_scan.angle_min,
            scan_angle_max=self.latest_scan.angle_max,
            scan_angle_increment=self.latest_scan.angle_increment,
            odom_x=self.current_pose[0],
            odom_y=self.current_pose[1],
            odom_theta=self.current_pose[2],
            odom_vx=self.current_vel[0],
            odom_vy=self.current_vel[1],
            odom_vtheta=self.current_vel[2]
        )

        self.current_mission.frames.append(frame)
        self.last_record_time = now

        if len(self.current_mission.frames) % 50 == 1:
            self.get_logger().info(f'[DataMule] Recording frames: {len(self.current_mission.frames)}')

    def _navigate_to_waypoint(self):
        """Navigate toward current waypoint."""
        if not self.target_waypoint:
            self._wander()
            return

        x, y, theta = self.current_pose
        tx, ty = self.target_waypoint

        dx = tx - x
        dy = ty - y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.5:
            self.get_logger().info(f'[DataMule] Reached waypoint {self.target_waypoint}')
            if self.current_mission:
                self.current_mission.waypoints_visited.append(f'{tx},{ty}')

            if self.waypoint_queue:
                self.target_waypoint = self.waypoint_queue.pop(0)
                self.get_logger().info(f'[DataMule] Next waypoint: {self.target_waypoint}')
            else:
                self.get_logger().info('[DataMule] All waypoints visited, returning')
                self.state = MuleState.RETURNING
            return

        target_heading = math.atan2(dy, dx)
        self._drive_toward_heading(target_heading)

    def _navigate_to_dock(self):
        """Navigate back to dock."""
        x, y, theta = self.current_pose
        dx = self.dock_position[0] - x
        dy = self.dock_position[1] - y

        target_heading = math.atan2(dy, dx)
        self._drive_toward_heading(target_heading)

    def _drive_toward_heading(self, target_heading: float):
        """Drive toward a heading with obstacle avoidance."""
        cmd = Twist()

        obstacle_left, obstacle_center, obstacle_right = self._check_obstacles()

        if obstacle_center:
            if obstacle_left and not obstacle_right:
                cmd.angular.z = -self.angular_speed
            elif obstacle_right and not obstacle_left:
                cmd.angular.z = self.angular_speed
            else:
                cmd.linear.x = -self.linear_speed * 0.5
                cmd.angular.z = self.angular_speed
        else:
            x, y, theta = self.current_pose
            heading_error = self._normalize_angle(target_heading - theta)

            if abs(heading_error) > 0.3:
                cmd.angular.z = self.angular_speed if heading_error > 0 else -self.angular_speed
                cmd.linear.x = self.linear_speed * 0.3
            else:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = heading_error * 0.5

        self.cmd_pub.publish(cmd)

    def _wander(self):
        """Simple wandering behavior."""
        cmd = Twist()

        obstacle_left, obstacle_center, obstacle_right = self._check_obstacles()

        if obstacle_center:
            if not obstacle_right:
                cmd.angular.z = -self.angular_speed
            else:
                cmd.angular.z = self.angular_speed
        else:
            cmd.linear.x = self.linear_speed

        self.cmd_pub.publish(cmd)

    def _check_obstacles(self) -> tuple:
        """Check for obstacles in left, center, right sectors."""
        if not self.latest_scan:
            return False, False, False

        ranges = self.latest_scan.ranges
        n = len(ranges)
        if n == 0:
            return False, False, False

        third = n // 3

        def has_obstacle(sector_ranges):
            valid = [r for r in sector_ranges if r > 0.01 and r < float('inf')]
            if not valid:
                return False
            return min(valid) < self.obstacle_thresh

        right = has_obstacle(ranges[:third])
        center = has_obstacle(ranges[third:2*third])
        left = has_obstacle(ranges[2*third:])

        return left, center, right

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _transfer_data(self):
        """Transfer recorded data to dock."""
        if not self.current_mission:
            self.state = MuleState.IDLE
            return

        self.current_mission.end_time = time.time()

        # Calculate start and end seconds for filename: Scan[SS:SS].json
        start_sec = int(self.current_mission.start_time) % 60
        end_sec = int(self.current_mission.end_time) % 60
        filename = f'{self.data_dir}/Scan[{start_sec:02d}:{end_sec:02d}].json'

        # Handle potential filename collision by appending mission count
        if os.path.exists(filename):
            # Find next available number
            base = f'{self.data_dir}/Scan[{start_sec:02d}:{end_sec:02d}]'
            counter = 2
            while os.path.exists(f'{base}_{counter}.json'):
                counter += 1
            filename = f'{base}_{counter}.json'

        try:
            with open(filename, 'w') as f:
                f.write(self.current_mission.to_json())
            self.get_logger().info(
                f'[DataMule] Saved {len(self.current_mission.frames)} frames to {filename}'
            )
        except Exception as e:
            self.get_logger().error(f'[DataMule] Failed to save data: {e}')

        msg = String()
        msg.data = json.dumps({
            'type': 'transfer_complete',
            'module_id': self.module_id,
            'mission_id': self.current_mission.mission_id,
            'filename': filename,
            'frame_count': len(self.current_mission.frames),
            'duration': self.current_mission.end_time - self.current_mission.start_time
        })
        self.data_pub.publish(msg)

        self.get_logger().info('[DataMule] Data transfer complete')
        self.state = MuleState.COMPLETE

        self.create_timer(2.0, self._return_to_idle, callback_group=None)

    def _return_to_idle(self):
        """Reset to idle state and teleport back to spawn position."""
        self.current_mission = None

        # Teleport back to spawn position (sim only - "plugging in the wires")
        self._teleport_to_spawn()

        self.state = MuleState.IDLE
        self._start_heartbeat()
        self.get_logger().info('[DataMule] Ready for next mission')

    def _teleport_to_spawn(self):
        """
        Reset rover position after docking.

        In simulation: Teleports via Gazebo service.
        On hardware: Logs ready status (human operator handles physical docking).
        """
        # Check if we're in simulation mode
        use_sim_time = self.get_parameter('use_sim_time').value

        if not use_sim_time:
            # Hardware mode - human operator handles docking
            self.get_logger().info(
                f'[DataMule] At dock position ({self.current_pose[0]:.2f}, '
                f'{self.current_pose[1]:.2f}) - ready for manual docking'
            )
            self.get_logger().info('[DataMule] Plug in rover to transfer data and recharge')
            return

        # Simulation mode - teleport via Gazebo
        import subprocess

        # Convert yaw to quaternion
        qz = math.sin(self.spawn_yaw / 2.0)
        qw = math.cos(self.spawn_yaw / 2.0)

        # Build the gz service command to set pose
        # Service: /world/coven_world/set_pose
        req_str = (
            f'name: "{self.module_id}", '
            f'position: {{x: {self.spawn_x}, y: {self.spawn_y}, z: 0.15}}, '
            f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
        )

        cmd = [
            'gz', 'service', '-s', '/world/coven_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req_str
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(
                    f'[DataMule] Teleported to spawn ({self.spawn_x:.2f}, {self.spawn_y:.2f})'
                )
            else:
                self.get_logger().warn(f'[DataMule] Teleport failed: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('[DataMule] Teleport command timed out')
        except Exception as e:
            self.get_logger().warn(f'[DataMule] Teleport error: {e}')

    def _publish_status(self):
        """Broadcast current status."""
        msg = String()
        status = {
            'module_id': self.module_id,
            'state': self.state.value,
            'position': {'x': self.current_pose[0], 'y': self.current_pose[1]},
            'heading': self.current_pose[2],
        }

        if self.current_mission:
            status['mission_id'] = self.current_mission.mission_id
            status['frames_recorded'] = len(self.current_mission.frames)

        if self.target_waypoint:
            status['target'] = {'x': self.target_waypoint[0], 'y': self.target_waypoint[1]}

        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _start_heartbeat(self):
        """Start sending heartbeats."""
        if self.hb_timer is not None:
            return
        self.hb_timer = self.create_timer(self.hb_period, self._send_heartbeat)
        self.get_logger().info(f'[DataMule] Heartbeat started for {self.module_id}')

    def _stop_heartbeat(self):
        """Stop sending heartbeats."""
        if self.hb_timer is not None:
            self.hb_timer.cancel()
            self.hb_timer = None
            self.get_logger().info(f'[DataMule] Heartbeat stopped for {self.module_id}')

    def _send_heartbeat(self):
        """Send a single heartbeat message."""
        self.hb_seq += 1
        hb = common.Heartbeat(module_id=self.module_id, seq=self.hb_seq)
        self.hb_pub.publish(String(data=common.hb_encode(hb)))


def main(args=None):
    rclpy.init(args=args)
    node = DataMuleModule()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
