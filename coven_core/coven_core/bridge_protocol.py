"""
bridge_protocol.py — COVEN handshake and message protocol handler.

Dispatches incoming rover messages (colon-delimited and JSON) to the
appropriate handler: identify, verify, heartbeat, scan, odom, batch,
and task lifecycle messages. Instantiated by the RoverBridge node.

Author: Alexander Shultis
Date: January 2025
"""

from __future__ import annotations

import json
import math
import time
from typing import List, TYPE_CHECKING

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from coven_core.common import (
    get_witch_name, mark_witch_active, is_witch_known, get_witch_status,
)
from coven_core.task_auctioneer import (
    PayloadType, RoverStatus as AuctionRoverStatus,
)

if TYPE_CHECKING:
    from coven_core.rover_bridge import RoverBridge, ConnectedRover, RoverState


class ProtocolHandler:
    """Handles the COVEN protocol message dispatch and handlers."""

    def __init__(self, bridge: 'RoverBridge'):
        self._bridge = bridge

    async def process_message(self, rover: 'ConnectedRover', message: str):
        """Process a message from a rover."""
        from coven_core.rover_bridge import RoverState  # noqa: F811

        if message.startswith('{'):
            self._bridge.get_logger().info(f"JSON from {rover.module_id}: {message[:200]}...")
        else:
            self._bridge.get_logger().debug(f"Received from {rover.module_id}: {message[:100]}...")

        # Try JSON first (for complex messages)
        if message.startswith('{'):
            try:
                data = json.loads(message)
                await self._process_json_message(rover, data)
                return
            except json.JSONDecodeError:
                pass

        # Parse colon-delimited message
        parts = message.split(':')
        msg_type = parts[0] if parts else ""

        if msg_type == "IDENTIFY_REP":
            await self._handle_identify_rep(rover, parts)
        elif msg_type == "VERIFY_REP":
            await self._handle_verify_rep(rover, parts)
        elif msg_type == "HEARTBEAT":
            await self._handle_heartbeat(rover, parts)
        elif msg_type == "TASK_ACK":
            await self._handle_task_ack(rover, parts)
        elif msg_type == "TASK_START":
            await self._handle_task_start(rover, parts)
        elif msg_type == "TASK_COMPLETE":
            await self._handle_task_complete(rover, parts)
        else:
            self._bridge.get_logger().warn(f"Unknown message type: {msg_type}")

    async def _process_json_message(self, rover: 'ConnectedRover', data: dict):
        """Process a JSON message (scan, odom, or batch data)."""
        if "DataBatch" in data:
            batch_wrapper = data["DataBatch"]
            mission_id = batch_wrapper.get("mission_id", "unknown")
            batch_data = batch_wrapper.get("batch", {})
            await self._bridge.data_proc.handle_data_batch(rover, mission_id, batch_data)
        elif "ScanData" in data:
            scan_wrapper = data["ScanData"]
            scan_data = scan_wrapper.get("scan", scan_wrapper)
            await self._handle_scan_data(rover, scan_data)
        elif "OdomData" in data:
            odom_wrapper = data["OdomData"]
            odom_data = odom_wrapper.get("odom", odom_wrapper)
            await self._handle_odom_data(rover, odom_data)
        else:
            self._bridge.get_logger().warn(f"Unknown JSON message: {list(data.keys())}")

    async def _handle_identify_rep(self, rover: 'ConnectedRover', parts: List[str]):
        """Handle IDENTIFY_REP from rover.

        Implements the three reconnection scenarios:
        1. New rover (claims "new_witch"): Assign fresh witch name
        2. Returning rover (known name, within timeout): "Welcome back!"
        3. Returning rover (name was reassigned): "No you aren't. You are <new_name>!"
        """
        from coven_core.rover_bridge import RoverState  # noqa: F811

        if len(parts) < 6:
            self._bridge.get_logger().error(f"Malformed IDENTIFY_REP (need 6 parts): {parts}")
            return

        claimed_name = parts[1]
        rover.module_type = parts[2]
        rover.firmware = parts[3]
        rover.battery_level = float(parts[4]) / 100.0 if len(parts) > 4 else 1.0
        rover_status = parts[5]

        if rover_status != "OK":
            self._bridge.get_logger().warning(
                f"Rover reported status '{rover_status}' during identification"
            )

        old_id = rover.module_id
        final_name = None
        welcome_message = None

        if claimed_name == "new_witch" or not claimed_name:
            final_name = get_witch_name()
            welcome_message = f"I'm gonna call you {final_name}!"
            self._bridge.get_logger().info(
                f"New rover connected - assigning name: {final_name}"
            )
        elif is_witch_known(claimed_name):
            status = get_witch_status(claimed_name)
            if status == "missing":
                final_name = get_witch_name(returning_name=claimed_name)
                if final_name == claimed_name:
                    welcome_message = f"Welcome back {final_name}!"
                    self._bridge.get_logger().info(
                        f"Returning rover {final_name} reconnected - welcome back!"
                    )
                else:
                    welcome_message = f"No you aren't. You are {final_name}!"
                    self._bridge.get_logger().info(
                        f"Rover claimed {claimed_name} but assigned {final_name}"
                    )
            elif status == "active":
                self._bridge.get_logger().warning(
                    f"Rover claims to be {claimed_name} but that name is already active!"
                )
                final_name = get_witch_name()
                welcome_message = f"No you aren't. You are {final_name}!"
            else:
                final_name = get_witch_name()
                welcome_message = f"No you aren't. You are {final_name}!"
                self._bridge.get_logger().info(
                    f"Rover claimed {claimed_name} (released), now {final_name}"
                )
        else:
            final_name = get_witch_name()
            welcome_message = f"No you aren't. You are {final_name}!"
            self._bridge.get_logger().info(
                f"Rover claimed unknown/expired name '{claimed_name}', assigned {final_name}"
            )

        rover.module_id = final_name
        rover.state = RoverState.IDENTIFIED

        mark_witch_active(final_name)

        self._bridge.get_logger().info(
            f"{rover.module_id} reporting for duty! ({rover.module_type}, fw {rover.firmware}, battery {rover.battery_level*100:.0f}%)"
        )

        with self._bridge.rovers_lock:
            if old_id in self._bridge.rovers and old_id != final_name:
                del self._bridge.rovers[old_id]
            self._bridge.rovers[rover.module_id] = rover

        self._bridge.topics.setup_rover(rover.module_id)

        await self._bridge._send_message(
            rover,
            f"IDENTIFY_ACK:{self._bridge.dock_id}:{final_name}:{welcome_message}"
        )

        await self._bridge._send_message(rover, f"VERIFY_REQ:{self._bridge.dock_id}:{rover.module_id}")

    async def _handle_verify_rep(self, rover: 'ConnectedRover', parts: List[str]):
        """Handle VERIFY_REP from rover."""
        from coven_core.rover_bridge import RoverState  # noqa: F811

        if len(parts) < 3:
            return

        success = parts[2].lower() == "true"

        if success:
            rover.state = RoverState.VERIFIED
            self._bridge.get_logger().info(f"Rover {rover.module_id} verified successfully")

            payload = PayloadType.LIDAR
            if "spectro" in rover.module_type.lower():
                payload = PayloadType.SPECTROMETER
            elif "drill" in rover.module_type.lower():
                payload = PayloadType.DRILL
            elif "cargo" in rover.module_type.lower():
                payload = PayloadType.CARGO
            elif "camera" in rover.module_type.lower():
                payload = PayloadType.CAMERA

            self._bridge.auctioneer.register_rover(
                module_id=rover.module_id,
                payload=payload,
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
            failed_checks = parts[3] if len(parts) > 3 else "unknown"
            self._bridge.get_logger().warn(f"Rover {rover.module_id} verification failed: {failed_checks}")

    async def _handle_heartbeat(self, rover: 'ConnectedRover', parts: List[str]):
        """Handle HEARTBEAT from rover."""
        if len(parts) < 7:
            rover.last_heartbeat = time.time()
            return

        rover.battery_level = float(parts[2]) / 100.0
        mission_status = parts[3]
        rover.x = float(parts[4])
        rover.y = float(parts[5])
        rover.theta = float(parts[6])
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

    async def _handle_scan_data(self, rover: 'ConnectedRover', scan_data: dict):
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

    async def _handle_odom_data(self, rover: 'ConnectedRover', odom_data: dict):
        """Handle odometry data from rover, publish as Odometry."""
        if rover.module_id not in self._bridge.topics.odom_pubs:
            self._bridge.get_logger().warn(
                f"No odom pub for {rover.module_id}, available: {list(self._bridge.topics.odom_pubs.keys())}"
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

    async def _handle_task_ack(self, rover: 'ConnectedRover', parts: List[str]):
        """Handle TASK_ACK from rover."""
        self._bridge._publish_event("TASK_ACK", rover.module_id, {"parts": parts})

    async def _handle_task_start(self, rover: 'ConnectedRover', parts: List[str]):
        """Handle TASK_START from rover."""
        self._bridge._publish_event("TASK_START", rover.module_id, {"parts": parts})

    async def _handle_task_complete(self, rover: 'ConnectedRover', parts: List[str]):
        """Handle TASK_COMPLETE from rover."""
        task_id = parts[2] if len(parts) > 2 else "unknown"
        success = parts[3].lower() == "true" if len(parts) > 3 else False
        duration = float(parts[6]) if len(parts) > 6 else 0.0

        rover.current_mission_id = None
        rover.mission_estimated_path_m = 0.0

        self._bridge.auctioneer.handle_task_complete(
            module_id=rover.module_id,
            task_id=task_id,
            success=success,
            duration=duration,
        )

        self._bridge._publish_event("TASK_COMPLETE", rover.module_id, {
            "parts": parts,
            "task_id": task_id,
            "success": success,
            "duration": duration,
        })
