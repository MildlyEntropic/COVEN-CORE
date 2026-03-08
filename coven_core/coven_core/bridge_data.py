# SPDX-License-Identifier: MIT
"""
bridge_data.py — Sensor batch processing and disk persistence.

Handles batch data uploads from rovers returning to dock, converts
raw encoder ticks to odometry, persists data to disk, and notifies
the offline SLAM processor. Instantiated by the RoverBridge node.

Author: Alexander Shultis
Date: January 2025
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import struct
from datetime import datetime
from typing import Optional, TYPE_CHECKING

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from coven_core.frame_codec import SENSOR_TYPE_LIDAR

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from coven_core.rover_bridge import RoverBridge, ConnectedRover


# Minimum samples for a valid batch
MIN_SAMPLES = 10
MAX_SAVE_RETRIES = 3


def _decode_lidar_sample(sample: dict, num_rays: int) -> list:
    """Extract LiDAR ranges (u16 mm) from a sample.

    Handles both formats:
    - Binary batch: sample["sensor_data"] is bytes (u16 LE array)
    - Legacy JSON batch: sample["lidar_ranges_mm"] is a list of ints
    """
    # Binary batch path
    sensor_data = sample.get("sensor_data")
    if isinstance(sensor_data, (bytes, bytearray)) and len(sensor_data) >= 2:
        count = min(len(sensor_data) // 2, num_rays)
        return list(struct.unpack_from(f'<{count}H', sensor_data))

    # Legacy JSON path
    return sample.get("lidar_ranges_mm", [])


class DataBatchProcessor:
    """Processes batch data uploads from rovers and persists to disk."""

    def __init__(self, bridge: 'RoverBridge'):
        self._bridge = bridge

    def handle_data_batch_sync(
        self, rover: 'ConnectedRover', mission_id: str, batch: dict,
        sensor_type: Optional[int] = None, sensor_config: Optional[bytes] = None,
    ):
        """Synchronous wrapper for handle_data_batch (called from serial thread)."""
        import asyncio
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(
                self.handle_data_batch(rover, mission_id, batch, sensor_type, sensor_config)
            )
        finally:
            loop.close()

    async def handle_data_batch(
        self, rover: 'ConnectedRover', mission_id: str, batch: dict,
        sensor_type: Optional[int] = None, sensor_config: Optional[bytes] = None,
    ):
        """Handle batch data upload from rover returning to dock.

        The rover is 'dumb' — it just collects raw encoder ticks and sensor data.
        The dock is smart — it checks sensor_type and knows how to interpret the
        sensor payload (LiDAR → SLAM, future sensors → appropriate processors).
        """
        samples = batch.get("samples", [])
        num_samples = len(samples)

        self._bridge.get_logger().info(
            f"Received data batch from {rover.module_id}: mission={mission_id}, "
            f"{num_samples} samples, sensor_type=0x{sensor_type or 0:02X}"
        )

        # === BATCH VALIDATION ===
        if num_samples < MIN_SAMPLES:
            self._bridge.get_logger().error(
                f"REJECTING batch from {rover.module_id}: only {num_samples} samples "
                f"(minimum {MIN_SAMPLES} required). Batch may be corrupted or mission too short."
            )
            self._bridge._publish_event("DATA_BATCH_REJECTED", rover.module_id, {
                "mission_id": mission_id,
                "num_samples": num_samples,
                "reason": f"insufficient_samples (need {MIN_SAMPLES}, got {num_samples})",
            })
            return

        required_fields = ["wheel_radius_mm", "wheel_base_mm", "ticks_per_rev"]
        missing_fields = [f for f in required_fields if f not in batch or batch[f] <= 0]
        if missing_fields:
            self._bridge.get_logger().error(
                f"REJECTING batch from {rover.module_id}: missing or invalid fields: {missing_fields}"
            )
            self._bridge._publish_event("DATA_BATCH_REJECTED", rover.module_id, {
                "mission_id": mission_id,
                "reason": f"missing_fields: {missing_fields}",
            })
            return

        # Extract robot configuration
        wheel_radius = batch.get("wheel_radius_mm", 80) / 1000.0
        wheel_base = batch.get("wheel_base_mm", 298) / 1000.0
        ticks_per_rev = batch.get("ticks_per_rev", 1440)

        # Decode sensor-specific config
        lidar_angle_min = -math.pi
        lidar_angle_max = math.pi
        lidar_num_rays = 360
        is_lidar = (sensor_type == SENSOR_TYPE_LIDAR) if sensor_type is not None else True

        if is_lidar and sensor_config and len(sensor_config) >= 18:
            # Binary batch: decode LiDAR config from opaque blob
            lidar_angle_min = struct.unpack_from('<d', sensor_config, 0)[0]
            lidar_angle_max = struct.unpack_from('<d', sensor_config, 8)[0]
            lidar_num_rays = struct.unpack_from('<H', sensor_config, 16)[0]
        elif is_lidar:
            # Legacy JSON batch: LiDAR config in batch dict
            lidar_angle_min = batch.get("lidar_angle_min", -math.pi)
            lidar_angle_max = batch.get("lidar_angle_max", math.pi)
            lidar_num_rays = batch.get("lidar_num_rays", 360)

        lidar_angle_increment = (lidar_angle_max - lidar_angle_min) / max(lidar_num_rays - 1, 1)

        if sensor_type is not None and not is_lidar:
            self._bridge.get_logger().warning(
                f"Unknown sensor_type 0x{sensor_type:02X} from {rover.module_id} — "
                "processing odometry only, saving raw sensor data to disk"
            )

        # Calculate meters per tick
        wheel_circumference = 2.0 * math.pi * wheel_radius
        meters_per_tick = wheel_circumference / ticks_per_rev

        # Process samples — convert raw ticks to odometry (computed once, reused for disk save)
        x, y, theta = 0.0, 0.0, 0.0
        computed_poses = []

        for sample in samples:
            left_ticks = sample.get("left_ticks", 0)
            right_ticks = sample.get("right_ticks", 0)

            dist_left = left_ticks * meters_per_tick
            dist_right = right_ticks * meters_per_tick
            dist_center = (dist_left + dist_right) / 2.0
            delta_theta = (dist_right - dist_left) / wheel_base

            theta_mid = theta + delta_theta / 2.0
            x += dist_center * math.cos(theta_mid)
            y += dist_center * math.sin(theta_mid)
            theta += delta_theta

            while theta > math.pi:
                theta -= 2.0 * math.pi
            while theta < -math.pi:
                theta += 2.0 * math.pi

            computed_poses.append((x, y, theta))

            # Create and publish Odometry message
            odom = Odometry()
            odom.header.stamp = self._bridge.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = f"{rover.module_id}/base_link"

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)

            self._bridge.topics.publish_odom_threadsafe(rover.module_id, odom)

            # Decode and publish sensor data based on type
            if is_lidar:
                lidar_ranges_mm = _decode_lidar_sample(sample, lidar_num_rays)
                if lidar_ranges_mm:
                    scan = LaserScan()
                    scan.header.stamp = odom.header.stamp
                    scan.header.frame_id = f"{rover.module_id}/laser_frame"

                    scan.angle_min = lidar_angle_min
                    scan.angle_max = lidar_angle_max
                    scan.angle_increment = lidar_angle_increment
                    scan.range_min = 0.12
                    scan.range_max = 10.0

                    scan.ranges = [
                        float(r) / 1000.0 if r > 0 else 0.0
                        for r in lidar_ranges_mm
                    ]

                    self._bridge.topics.publish_scan_threadsafe(rover.module_id, scan)

        # Update rover position from final sample
        rover.x = x
        rover.y = y
        rover.theta = theta

        # Save batch data to disk with retries
        saved_filename = None

        for attempt in range(1, MAX_SAVE_RETRIES + 1):
            saved_filename = self._save_batch_to_disk(
                rover, mission_id, batch, samples, computed_poses,
                lidar_angle_min, lidar_angle_max, lidar_angle_increment,
                sensor_type, sensor_config,
            )
            if saved_filename:
                break
            self._bridge.get_logger().warning(
                f"Save attempt {attempt}/{MAX_SAVE_RETRIES} failed for batch from {rover.module_id}"
            )
            if attempt < MAX_SAVE_RETRIES:
                await asyncio.sleep(0.5)

        # Notify offline SLAM processor if saved successfully
        if saved_filename:
            self._notify_slam_processor(
                rover.module_id, mission_id, saved_filename, len(samples)
            )
            self._bridge._publish_event("DATA_BATCH_RECEIVED", rover.module_id, {
                "mission_id": mission_id,
                "num_samples": len(samples),
                "final_x": x,
                "final_y": y,
                "final_theta": theta,
                "saved_file": saved_filename,
            })
        else:
            self._bridge.get_logger().error(
                f"FAILED to save batch from {rover.module_id} after {MAX_SAVE_RETRIES} attempts. "
                "Data lost - SLAM will not process this mission."
            )
            self._bridge._publish_event("DATA_BATCH_SAVE_FAILED", rover.module_id, {
                "mission_id": mission_id,
                "num_samples": len(samples),
                "reason": "file_save_failed_after_retries",
            })

        self._bridge.get_logger().info(
            f"Processed batch from {rover.module_id}: final pose x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}°"
        )

    def _save_batch_to_disk(
        self, rover: 'ConnectedRover', mission_id: str, batch: dict,
        samples: list, computed_poses: list,
        lidar_angle_min: float, lidar_angle_max: float, lidar_angle_increment: float,
        sensor_type: Optional[int] = None, sensor_config: Optional[bytes] = None,
    ) -> Optional[str]:
        """Save processed batch data to disk.

        For LiDAR batches: produces JSON frames for offline SLAM processor.
        For unknown sensor types: saves raw data so nothing is lost.

        Returns the saved filename on success, None on failure.
        """
        rover_dir = os.path.join(self._bridge.session_data_dir, rover.module_id)
        os.makedirs(rover_dir, exist_ok=True)

        # Calculate elapsed time since session start
        elapsed = datetime.now() - self._bridge.session_start
        elapsed_days = elapsed.days
        elapsed_secs = elapsed.seconds
        elapsed_hours = elapsed_secs // 3600
        elapsed_mins = (elapsed_secs % 3600) // 60
        elapsed_sec = elapsed_secs % 60
        elapsed_str = f"{elapsed_days}.{elapsed_hours:02d}{elapsed_mins:02d}.{elapsed_sec:02d}"

        filename = os.path.join(rover_dir, f"Scan.{elapsed_str}.json")

        is_lidar = (sensor_type == SENSOR_TYPE_LIDAR) if sensor_type is not None else True
        lidar_num_rays = 360
        if is_lidar and sensor_config and len(sensor_config) >= 18:
            lidar_num_rays = struct.unpack_from('<H', sensor_config, 16)[0]

        # Convert samples to frames using pre-computed poses (no recomputation)
        frames = []

        for sample, (x, y, theta) in zip(samples, computed_poses):
            frame = {
                "timestamp": sample.get("timestamp", 0.0),
                "odom_x": x,
                "odom_y": y,
                "odom_theta": theta,
                "odom_vx": 0.0,
                "odom_vy": 0.0,
                "odom_vtheta": 0.0,
            }

            if is_lidar:
                lidar_ranges_mm = _decode_lidar_sample(sample, lidar_num_rays)
                scan_ranges = [
                    float(r) / 1000.0 if r > 0 else 0.0
                    for r in lidar_ranges_mm
                ]
                frame["scan_angle_min"] = lidar_angle_min
                frame["scan_angle_max"] = lidar_angle_max
                frame["scan_angle_increment"] = lidar_angle_increment
                frame["scan_ranges"] = scan_ranges
            else:
                # Unknown sensor: save raw sensor_data as hex
                raw = sample.get("sensor_data")
                if isinstance(raw, (bytes, bytearray)):
                    frame["sensor_data_hex"] = raw.hex()
                    frame["sensor_type"] = sensor_type

            frames.append(frame)

        mission_start_ts = batch.get("mission_start", 0.0)
        first_ts = samples[0].get("timestamp", 0.0) if samples else 0.0
        last_ts = samples[-1].get("timestamp", 0.0) if samples else 0.0
        mission_duration = last_ts - first_ts

        mission_data = {
            "mission_id": mission_id,
            "module_id": rover.module_id,
            "start_time": mission_start_ts,
            "end_time": mission_start_ts + mission_duration,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "initial_theta": 0.0,
            "frames": frames,
            "metadata": {
                "wheel_radius_m": batch.get("wheel_radius_mm", 80) / 1000.0,
                "wheel_base_m": batch.get("wheel_base_mm", 298) / 1000.0,
                "ticks_per_rev": batch.get("ticks_per_rev", 1440),
                "dock_id": self._bridge.dock_id,
                "coven_name": self._bridge.coven_name,
                "sensor_type": sensor_type,
            }
        }

        try:
            with open(filename, 'w') as f:
                json.dump(mission_data, f, indent=2)
            self._bridge.get_logger().info(f"Saved batch data to {filename}")
            return filename
        except Exception as e:
            self._bridge.get_logger().error(f"Failed to save batch data: {e}")
            return None

    def _notify_slam_processor(
        self, module_id: str, mission_id: str, filename: str, frame_count: int
    ):
        """Notify the offline SLAM processor that new sensor data is available."""
        msg = String()
        msg.data = json.dumps({
            "type": "transfer_complete",
            "filename": filename,
            "mission_id": mission_id,
            "module_id": module_id,
            "frame_count": frame_count,
        })
        self._bridge.sensor_batch_pub.publish(msg)
        self._bridge.get_logger().info(
            f"Notified SLAM processor: {mission_id} ({frame_count} frames)"
        )
