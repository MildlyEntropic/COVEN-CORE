# SPDX-License-Identifier: MIT
"""
bridge_protocol.py — COVEN handshake and message protocol handler.

Dispatches decoded binary frame messages to the appropriate handler:
identify, verify, heartbeat, scan, odom, batch, and task lifecycle.
Messages arrive as Python dicts from frame_codec.decode_message().
Instantiated by the RoverBridge node.

Author: Alexander Shultis
Date: January 2025
"""

from __future__ import annotations

import json
import math
import time
from typing import TYPE_CHECKING

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from coven_core.common import (
    get_witch_name, mark_witch_active, is_witch_known, get_witch_status,
)
from coven_core.task_auctioneer import (
    PayloadType, RoverStatus as AuctionRoverStatus,
)
from coven_core.frame_codec import (
    encode_identify_request,
    encode_identify_ack,
    encode_verify_ok,
    encode_verify_fail,
    BatchChunkAssembler,
    CAP_LIDAR,
    CAP_CAMERA,
    CAP_SPECTROMETER,
    CAP_DRILL,
    capabilities_to_list,
)

if TYPE_CHECKING:
    from coven_core.rover_bridge import RoverBridge, ConnectedRover, RoverState


class ProtocolHandler:
    """Handles the COVEN protocol message dispatch and handlers."""

    def __init__(self, bridge: 'RoverBridge'):
        self._bridge = bridge
        self._batch_assembler = BatchChunkAssembler(timeout_secs=30.0)

    def process_message(self, rover: 'ConnectedRover', msg: dict):
        """Process a decoded message dict from frame_codec.decode_message()."""
        from coven_core.rover_bridge import RoverState  # noqa: F811

        msg_type = msg.get("type", "")

        self._bridge.get_logger().debug(
            f"Received from {rover.module_id}: {msg_type}"
        )

        if msg_type == "IDENTIFY_REPLY":
            self._handle_identify_reply(rover, msg)
        elif msg_type == "VERIFY_REP":
            self._handle_verify_rep(rover, msg)
        elif msg_type == "HEARTBEAT":
            self._handle_heartbeat(rover, msg)
        elif msg_type == "TASK_ACK":
            self._handle_task_ack(rover, msg)
        elif msg_type == "TASK_START":
            self._handle_task_start(rover, msg)
        elif msg_type == "TASK_COMPLETE":
            self._handle_task_complete(rover, msg)
        elif msg_type == "DATA_BATCH":
            self._handle_data_batch(rover, msg)
        elif msg_type == "SENSOR_BATCH_CHUNK":
            self._handle_sensor_batch_chunk(rover, msg)
        elif msg_type == "SCAN_DATA":
            self._handle_scan_data_wrapper(rover, msg)
        elif msg_type == "ODOM_DATA":
            self._handle_odom_data_wrapper(rover, msg)
        elif msg_type == "FAULT_ALERT":
            self._bridge.get_logger().error(
                f"FAULT from {rover.module_id}: {msg.get('payload', 'unknown')}"
            )
        elif msg_type == "SYSTEM_PING":
            self._bridge.get_logger().debug(f"PING from {rover.module_id}")
        else:
            self._bridge.get_logger().warning(f"Unknown message type: {msg_type}")

    # -------------------------------------------------------------------------
    # Handshake
    # -------------------------------------------------------------------------

    def _handle_identify_reply(self, rover: 'ConnectedRover', msg: dict):
        """Handle IDENTIFY_REPLY from rover.

        Implements the three reconnection scenarios:
        1. New rover (claims "new_witch"): Assign fresh witch name
        2. Returning rover (known name, within timeout): "Welcome back!"
        3. Returning rover (name was reassigned): "No you aren't. You are <new_name>!"
        """
        from coven_core.rover_bridge import RoverState  # noqa: F811

        claimed_name = msg.get("module_id", "")
        rover.module_type = msg.get("module_type", "unknown")
        rover.firmware = msg.get("firmware", "unknown")
        rover.battery_level = msg.get("battery_level", 100.0) / 100.0
        rover.capabilities = msg.get("capabilities", 0)
        rover_status = msg.get("status", "OK")

        if rover_status != "OK":
            self._bridge.get_logger().warning(
                f"Rover reported status '{rover_status}' during identification"
            )

        old_id = rover.module_id
        final_name = None

        if claimed_name == "new_witch" or not claimed_name:
            final_name = get_witch_name()
            self._bridge.get_logger().info(
                f"New rover connected — assigning name: {final_name}"
            )
        elif is_witch_known(claimed_name):
            status = get_witch_status(claimed_name)
            if status == "missing":
                final_name = get_witch_name(returning_name=claimed_name)
                if final_name == claimed_name:
                    self._bridge.get_logger().info(
                        f"Returning rover {final_name} reconnected — welcome back!"
                    )
                else:
                    self._bridge.get_logger().info(
                        f"Rover claimed {claimed_name} but assigned {final_name}"
                    )
            elif status == "active":
                self._bridge.get_logger().warning(
                    f"Rover claims to be {claimed_name} but that name is already active!"
                )
                final_name = get_witch_name()
            else:
                final_name = get_witch_name()
                self._bridge.get_logger().info(
                    f"Rover claimed {claimed_name} (released), now {final_name}"
                )
        else:
            final_name = get_witch_name()
            self._bridge.get_logger().info(
                f"Rover claimed unknown/expired name '{claimed_name}', assigned {final_name}"
            )

        rover.module_id = final_name
        rover.state = RoverState.IDENTIFIED

        mark_witch_active(final_name)

        caps = capabilities_to_list(rover.capabilities)
        self._bridge.get_logger().info(
            f"{rover.module_id} reporting for duty! "
            f"({rover.module_type}, fw {rover.firmware}, "
            f"battery {rover.battery_level * 100:.0f}%, "
            f"caps={caps})"
        )

        with self._bridge.rovers_lock:
            if old_id in self._bridge.rovers and old_id != final_name:
                del self._bridge.rovers[old_id]
            self._bridge.rovers[rover.module_id] = rover

        self._bridge.topics.setup_rover(rover.module_id)

        # Send IDENTIFY_ACK confirming the rover's final name
        self._bridge.send_frame(
            encode_identify_ack(self._bridge.dock_id, rover.module_id)
        )

        # Then send VERIFY_OK to proceed with verification
        self._bridge.send_frame(
            encode_verify_ok(self._bridge.dock_id, rover.module_id)
        )

    def _handle_verify_rep(self, rover: 'ConnectedRover', msg: dict):
        """Handle VERIFY_REP from rover (DATA_FRAME subtype 0x01)."""
        from coven_core.rover_bridge import RoverState  # noqa: F811

        success = msg.get("success", False)

        if success:
            rover.state = RoverState.VERIFIED
            self._bridge.get_logger().info(f"Rover {rover.module_id} verified successfully")

            # Infer primary payload from capability bitmask
            if rover.capabilities & CAP_LIDAR:
                payload = PayloadType.LIDAR
            elif rover.capabilities & CAP_CAMERA:
                payload = PayloadType.CAMERA
            elif rover.capabilities & CAP_SPECTROMETER:
                payload = PayloadType.SPECTROMETER
            elif rover.capabilities & CAP_DRILL:
                payload = PayloadType.DRILL
            else:
                payload = PayloadType.CARGO  # Encoders-only rover

            self._bridge.auctioneer.register_rover(
                module_id=rover.module_id,
                payload=payload,
                capabilities=rover.capabilities,
                battery_pct=rover.battery_level * 100.0,
                position=(rover.x, rover.y),
                heading=rover.theta,
            )
            self._bridge.auctioneer.update_rover_status(
                module_id=rover.module_id,
                status=AuctionRoverStatus.IDLE,
            )

            self._bridge._publish_event("ROVER_VERIFIED", rover.module_id, {
                "payload": payload.value,
            })

            rover.state = RoverState.ACTIVE
        else:
            failed = msg.get("failed_checks", [])
            note = msg.get("note", "")
            self._bridge.get_logger().warning(
                f"Rover {rover.module_id} verification failed: {failed} ({note})"
            )

    # -------------------------------------------------------------------------
    # Heartbeat
    # -------------------------------------------------------------------------

    def _handle_heartbeat(self, rover: 'ConnectedRover', msg: dict):
        """Handle MODULE_HEARTBEAT from rover."""
        rover.battery_level = msg.get("battery_pct", 100.0) / 100.0
        mission_status = msg.get("mission_status", "IDLE")
        rover.x = msg.get("x", 0.0)
        rover.y = msg.get("y", 0.0)
        rover.theta = msg.get("theta", 0.0)
        rover.last_heartbeat = time.time()

        auction_status = AuctionRoverStatus.IDLE
        if mission_status == "ACTIVE":
            auction_status = AuctionRoverStatus.ACTIVE
        elif mission_status == "STARTUP":
            auction_status = AuctionRoverStatus.UNKNOWN

        self._bridge.auctioneer.update_rover_status(
            module_id=rover.module_id,
            status=auction_status,
            battery_pct=rover.battery_level * 100.0,
            position=(rover.x, rover.y),
            heading=rover.theta,
        )

        self._bridge._publish_rover_status(rover, mission_status)
        self._bridge.topics.publish_rover_tf(rover)

    # -------------------------------------------------------------------------
    # Sensor Data
    # -------------------------------------------------------------------------

    def _handle_scan_data_wrapper(self, rover: 'ConnectedRover', msg: dict):
        """Handle ScanData from DATA_FRAME subtype 0x20 (JSON)."""
        scan_wrapper = msg.get("ScanData", msg)
        scan_data = scan_wrapper.get("scan", scan_wrapper)
        self._handle_scan_data(rover, scan_data)

    def _handle_scan_data(self, rover: 'ConnectedRover', scan_data: dict):
        """Handle scan data from rover, publish as LaserScan."""
        if rover.module_id not in self._bridge.topics.scan_pubs:
            return

        scan = LaserScan()
        scan.header.stamp = self._bridge.get_clock().now().to_msg()
        scan.header.frame_id = f"{rover.module_id}/laser_frame"

        scan.angle_min = float(scan_data.get("angle_min", -math.pi))
        scan.angle_max = float(scan_data.get("angle_max", math.pi))
        scan.angle_increment = float(scan_data.get("angle_increment", 0.01))
        scan.range_min = float(scan_data.get("range_min", 0.12))
        scan.range_max = float(scan_data.get("range_max", 10.0))

        ranges_mm = scan_data.get("ranges_mm", [])
        scan.ranges = [
            float(r) / 1000.0 if r > 0 else float('inf')
            for r in ranges_mm
        ]

        if len(scan.ranges) > 0:
            scan.time_increment = (1.0 / 6.0) / len(scan.ranges)
            scan.scan_time = 1.0 / 6.0

        self._bridge.topics.publish_scan_threadsafe(rover.module_id, scan)
        rover.last_scan = scan

    def _handle_odom_data_wrapper(self, rover: 'ConnectedRover', msg: dict):
        """Handle OdomData from DATA_FRAME subtype 0x20 (JSON)."""
        odom_wrapper = msg.get("OdomData", msg)
        odom_data = odom_wrapper.get("odom", odom_wrapper)
        self._handle_odom_data(rover, odom_data)

    def _handle_odom_data(self, rover: 'ConnectedRover', odom_data: dict):
        """Handle odometry data from rover, publish as Odometry."""
        if rover.module_id not in self._bridge.topics.odom_pubs:
            self._bridge.get_logger().warning(
                f"No odom pub for {rover.module_id}, "
                f"available: {list(self._bridge.topics.odom_pubs.keys())}"
            )
            return

        odom = Odometry()
        odom.header.stamp = self._bridge.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = f"{rover.module_id}/base_link"

        odom.pose.pose.position.x = float(odom_data.get("x", 0.0))
        odom.pose.pose.position.y = float(odom_data.get("y", 0.0))
        odom.pose.pose.position.z = 0.0

        theta = float(odom_data.get("theta", 0.0))
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        odom.twist.twist.linear.x = float(odom_data.get("v_linear", 0.0))
        odom.twist.twist.angular.z = float(odom_data.get("v_angular", 0.0))

        self._bridge.topics.publish_odom_threadsafe(rover.module_id, odom)
        rover.last_odom = odom

        rover.x = odom.pose.pose.position.x
        rover.y = odom.pose.pose.position.y
        rover.theta = theta

        self._bridge.topics.publish_rover_tf(rover)

    # -------------------------------------------------------------------------
    # Data Batch
    # -------------------------------------------------------------------------

    def _handle_data_batch(self, rover: 'ConnectedRover', msg: dict):
        """Handle DataBatch — from JSON (subtype 0x20) or binary reassembly (0x21)."""
        mission_id = msg.get("mission_id") or msg.get("DataBatch", {}).get("mission_id", "unknown")
        batch_data = msg.get("batch") or msg.get("DataBatch", {}).get("batch", msg)
        self._bridge.data_proc.handle_data_batch_sync(
            rover, mission_id, batch_data,
            sensor_type=msg.get("sensor_type"),
            sensor_config=msg.get("sensor_config"),
        )

    def _handle_sensor_batch_chunk(self, rover: 'ConnectedRover', msg: dict):
        """Handle a binary sensor batch chunk (subtype 0x21).

        Feeds the chunk to the assembler. When all chunks arrive,
        dispatches the complete batch to _handle_data_batch.
        """
        chunk_data = msg.get("chunk_data", b"")
        result = self._batch_assembler.feed_chunk(chunk_data)

        if result is not None:
            num_samples = len(result.get("batch", {}).get("samples", []))
            self._bridge.get_logger().info(
                f"Batch reassembly complete from {rover.module_id}: "
                f"{num_samples} samples"
            )
            self._handle_data_batch(rover, result)

    # -------------------------------------------------------------------------
    # Task Lifecycle
    # -------------------------------------------------------------------------

    def _handle_task_ack(self, rover: 'ConnectedRover', msg: dict):
        """Handle TASK_ACK from rover."""
        self._bridge._publish_event("TASK_ACK", rover.module_id, msg)

    def _handle_task_start(self, rover: 'ConnectedRover', msg: dict):
        """Handle TASK_START from rover."""
        self._bridge._publish_event("TASK_START", rover.module_id, msg)

    def _handle_task_complete(self, rover: 'ConnectedRover', msg: dict):
        """Handle TASK_COMPLETE from rover."""
        task_id = msg.get("task_id", "unknown")
        success = msg.get("success", False)
        duration = msg.get("duration", 0.0)

        rover.current_mission_id = None
        rover.mission_estimated_path_m = 0.0

        self._bridge.auctioneer.handle_task_complete(
            module_id=rover.module_id,
            task_id=task_id,
            success=success,
            duration=duration,
        )

        self._bridge._publish_event("TASK_COMPLETE", rover.module_id, {
            "task_id": task_id,
            "success": success,
            "duration": duration,
        })
