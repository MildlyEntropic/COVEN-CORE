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
import math
import os
from datetime import datetime
from typing import Optional, TYPE_CHECKING

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

if TYPE_CHECKING:
    from coven_core.rover_bridge import RoverBridge, ConnectedRover


# Minimum samples for a valid batch
MIN_SAMPLES = 10
MAX_SAVE_RETRIES = 3


class DataBatchProcessor:
    """Processes batch data uploads from rovers and persists to disk."""

    def __init__(self, bridge: 'RoverBridge'):
        self._bridge = bridge

    def handle_data_batch_sync(
        self, rover: 'ConnectedRover', mission_id: str, batch: dict
    ):
        """Synchronous wrapper for handle_data_batch (called from serial thread)."""
        import asyncio
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(self.handle_data_batch(rover, mission_id, batch))
        finally:
            loop.close()

    async def handle_data_batch(
        self, rover: 'ConnectedRover', mission_id: str, batch: dict
    ):
        """Handle batch data upload from rover returning to dock.

        The rover is 'dumb' — it just collects raw encoder ticks and LiDAR ranges.
        The dock converts the raw data to odometry and publishes it for SLAM processing.
        """
        samples = batch.get("samples", [])
        num_samples = len(samples)

        has_lidar = bool(rover.capabilities & 0x02) if hasattr(rover, 'capabilities') else True
        self._bridge.get_logger().info(
            f"Received data batch from {rover.module_id}: mission={mission_id}, "
            f"{num_samples} samples"
        )
        if not has_lidar:
            self._bridge.get_logger().info(
                f"Processing encoder-only batch from {rover.module_id} "
                "(no LiDAR data expected)"
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

        # LiDAR config
        lidar_angle_min = batch.get("lidar_angle_min", -math.pi)
        lidar_angle_max = batch.get("lidar_angle_max", math.pi)
        lidar_num_rays = batch.get("lidar_num_rays", 360)
        lidar_angle_increment = (lidar_angle_max - lidar_angle_min) / max(lidar_num_rays - 1, 1)

        # Calculate meters per tick
        wheel_circumference = 2.0 * math.pi * wheel_radius
        meters_per_tick = wheel_circumference / ticks_per_rev

        # Process samples — convert raw ticks to odometry
        x, y, theta = 0.0, 0.0, 0.0

        for sample in samples:
            left_ticks = sample.get("left_ticks", 0)
            right_ticks = sample.get("right_ticks", 0)
            lidar_ranges_mm = sample.get("lidar_ranges_mm", [])

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

            # Create and publish LaserScan message if we have LiDAR data
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
                rover, mission_id, batch, samples,
                wheel_radius, wheel_base, ticks_per_rev, meters_per_tick,
                lidar_angle_min, lidar_angle_max, lidar_angle_increment
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
        samples: list, wheel_radius: float, wheel_base: float,
        ticks_per_rev: int, meters_per_tick: float,
        lidar_angle_min: float, lidar_angle_max: float, lidar_angle_increment: float
    ) -> Optional[str]:
        """Save processed batch data to disk.

        Format: ~/Desktop/COVEN/Data/{YYYYMMDD.HHMM.SS}/{coven}/{rover}/Scan.{D.HHMM.SS}.json

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

        # Convert samples to frames format expected by offline_slam_processor
        frames = []
        x, y, theta = 0.0, 0.0, 0.0

        for sample in samples:
            left_ticks = sample.get("left_ticks", 0)
            right_ticks = sample.get("right_ticks", 0)
            lidar_ranges_mm = sample.get("lidar_ranges_mm", [])

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

            scan_ranges = [
                float(r) / 1000.0 if r > 0 else 0.0
                for r in lidar_ranges_mm
            ]

            frame = {
                "timestamp": sample.get("timestamp", 0.0),
                "odom_x": x,
                "odom_y": y,
                "odom_theta": theta,
                "odom_vx": 0.0,
                "odom_vy": 0.0,
                "odom_vtheta": 0.0,
                "scan_angle_min": lidar_angle_min,
                "scan_angle_max": lidar_angle_max,
                "scan_angle_increment": lidar_angle_increment,
                "scan_ranges": scan_ranges,
            }
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
                "wheel_radius_m": wheel_radius,
                "wheel_base_m": wheel_base,
                "ticks_per_rev": ticks_per_rev,
                "dock_id": self._bridge.dock_id,
                "coven_name": self._bridge.coven_name,
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
