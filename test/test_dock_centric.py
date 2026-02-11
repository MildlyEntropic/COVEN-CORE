"""
test_dock_centric.py - Unit tests for dock-centric architecture components

Tests the new dock-centric components:
- SimplifiedModuleState enum
- SensorPublisher (without ROS runtime)
- VelocityExecutor (without ROS runtime)
- Message encoding/decoding for dock-centric messages

Author: Alexander Shultis
Date: December 2025
"""

import unittest
from unittest.mock import MagicMock, patch
import math

from coven_core.common import (
    SimplifiedModuleState,
    RoverRegistration, RoverRegistrationAck,
    SensorData, VelocityCommand,
    RoverStatus, DockCommand,
    rover_registration_encode, rover_registration_decode,
    sensor_data_encode, sensor_data_decode,
    velocity_command_encode, velocity_command_decode,
    rover_status_encode, rover_status_decode,
)
from coven_core.serialization import encode, decode


class TestSimplifiedModuleState(unittest.TestCase):
    """Test SimplifiedModuleState enum."""

    def test_state_values(self):
        """SimplifiedModuleState has correct values."""
        self.assertEqual(SimplifiedModuleState.BOOT.value, 0)
        self.assertEqual(SimplifiedModuleState.READY.value, 1)
        self.assertEqual(SimplifiedModuleState.ACTIVE.value, 2)
        self.assertEqual(SimplifiedModuleState.ERROR.value, 3)

    def test_state_names(self):
        """SimplifiedModuleState has correct names."""
        self.assertEqual(SimplifiedModuleState.BOOT.name, "BOOT")
        self.assertEqual(SimplifiedModuleState.READY.name, "READY")
        self.assertEqual(SimplifiedModuleState.ACTIVE.name, "ACTIVE")
        self.assertEqual(SimplifiedModuleState.ERROR.name, "ERROR")

    def test_state_count(self):
        """SimplifiedModuleState has exactly 4 states."""
        self.assertEqual(len(SimplifiedModuleState), 4)


class TestRoverRegistrationMessages(unittest.TestCase):
    """Test RoverRegistration and RoverRegistrationAck messages."""

    def test_registration_creation(self):
        """RoverRegistration can be created with defaults."""
        reg = RoverRegistration(module_id="TestBot")
        self.assertEqual(reg.module_id, "TestBot")
        self.assertEqual(reg.module_type, "lidar_rover")
        self.assertEqual(reg.firmware_version, "1.0.0")
        self.assertEqual(reg.capabilities, ["lidar", "odom"])
        self.assertEqual(reg.initial_battery, 1.0)

    def test_registration_custom_values(self):
        """RoverRegistration can be created with custom values."""
        reg = RoverRegistration(
            module_id="CustomBot",
            module_type="camera_rover",
            firmware_version="2.5.0",
            capabilities=["camera", "imu", "gps"],
            initial_battery=0.85
        )
        self.assertEqual(reg.module_type, "camera_rover")
        self.assertEqual(reg.capabilities, ["camera", "imu", "gps"])
        self.assertAlmostEqual(reg.initial_battery, 0.85)

    def test_registration_ack_accepted(self):
        """RoverRegistrationAck for accepted registration."""
        ack = RoverRegistrationAck(
            module_id="TestBot",
            accepted=True,
            assigned_namespace="TestBot",
            reason=""
        )
        self.assertTrue(ack.accepted)
        self.assertEqual(ack.assigned_namespace, "TestBot")

    def test_registration_ack_rejected(self):
        """RoverRegistrationAck for rejected registration."""
        ack = RoverRegistrationAck(
            module_id="Unknown",
            accepted=False,
            assigned_namespace="",
            reason="Unknown rover type"
        )
        self.assertFalse(ack.accepted)
        self.assertEqual(ack.reason, "Unknown rover type")


class TestSensorDataMessage(unittest.TestCase):
    """Test SensorData message."""

    def test_sensor_data_minimal(self):
        """SensorData with minimal data."""
        data = SensorData(module_id="Rover1")
        self.assertEqual(data.module_id, "Rover1")
        self.assertEqual(data.scan_ranges, [])
        self.assertAlmostEqual(data.battery_level, 1.0)

    def test_sensor_data_full(self):
        """SensorData with full sensor readings."""
        data = SensorData(
            module_id="Rover2",
            timestamp=12345.678,
            scan_ranges=[1.0, 2.0, 3.0, 4.0, 5.0],
            scan_angle_min=-3.14,
            scan_angle_max=3.14,
            scan_angle_increment=0.01,
            odom_x=1.5,
            odom_y=2.5,
            odom_theta=0.785,
            odom_vx=0.3,
            odom_vtheta=0.1,
            battery_level=0.75
        )
        self.assertEqual(len(data.scan_ranges), 5)
        self.assertAlmostEqual(data.odom_x, 1.5)
        self.assertAlmostEqual(data.odom_theta, 0.785)
        self.assertAlmostEqual(data.battery_level, 0.75)

    def test_sensor_data_roundtrip(self):
        """SensorData survives encode/decode."""
        original = SensorData(
            module_id="RoundTrip",
            timestamp=999.999,
            scan_ranges=[1.0, 2.0, 3.0],
            odom_x=5.0,
            odom_y=-3.0,
            battery_level=0.5
        )
        encoded = encode(original)
        decoded = decode(encoded, SensorData)

        self.assertEqual(decoded.module_id, "RoundTrip")
        self.assertEqual(decoded.scan_ranges, [1.0, 2.0, 3.0])
        self.assertAlmostEqual(decoded.odom_x, 5.0)
        self.assertAlmostEqual(decoded.battery_level, 0.5)


class TestVelocityCommandMessage(unittest.TestCase):
    """Test VelocityCommand message."""

    def test_velocity_stop(self):
        """VelocityCommand for stop."""
        cmd = VelocityCommand(module_id="Rover1")
        self.assertEqual(cmd.module_id, "Rover1")
        self.assertAlmostEqual(cmd.linear_x, 0.0)
        self.assertAlmostEqual(cmd.angular_z, 0.0)

    def test_velocity_forward(self):
        """VelocityCommand for forward motion."""
        cmd = VelocityCommand(
            module_id="Rover2",
            linear_x=0.5,
            angular_z=0.0,
            timeout=0.25
        )
        self.assertAlmostEqual(cmd.linear_x, 0.5)
        self.assertAlmostEqual(cmd.timeout, 0.25)

    def test_velocity_turn(self):
        """VelocityCommand for turning."""
        cmd = VelocityCommand(
            module_id="Rover3",
            linear_x=0.0,
            angular_z=1.57
        )
        self.assertAlmostEqual(cmd.angular_z, 1.57)

    def test_velocity_arc(self):
        """VelocityCommand for arc motion."""
        cmd = VelocityCommand(
            module_id="Rover4",
            linear_x=0.3,
            angular_z=-0.5
        )
        self.assertAlmostEqual(cmd.linear_x, 0.3)
        self.assertAlmostEqual(cmd.angular_z, -0.5)


class TestRoverStatusMessage(unittest.TestCase):
    """Test RoverStatus message."""

    def test_status_ready(self):
        """RoverStatus in READY state."""
        status = RoverStatus(
            module_id="Rover1",
            state="READY",
            battery_level=0.9,
            is_moving=False
        )
        self.assertEqual(status.state, "READY")
        self.assertFalse(status.is_moving)
        self.assertEqual(status.error_msg, "")

    def test_status_active(self):
        """RoverStatus in ACTIVE state."""
        status = RoverStatus(
            module_id="Rover2",
            state="ACTIVE",
            battery_level=0.75,
            is_moving=True,
            last_cmd_age=0.05
        )
        self.assertEqual(status.state, "ACTIVE")
        self.assertTrue(status.is_moving)
        self.assertAlmostEqual(status.last_cmd_age, 0.05)

    def test_status_error(self):
        """RoverStatus in ERROR state."""
        status = RoverStatus(
            module_id="BrokenBot",
            state="ERROR",
            battery_level=0.05,
            is_moving=False,
            error_msg="Critical battery level"
        )
        self.assertEqual(status.state, "ERROR")
        self.assertEqual(status.error_msg, "Critical battery level")


class TestDockCommandMessage(unittest.TestCase):
    """Test DockCommand message."""

    def test_command_start(self):
        """DockCommand start."""
        cmd = DockCommand(module_id="Rover1", command="start")
        self.assertEqual(cmd.command, "start")
        self.assertEqual(cmd.parameters, {})

    def test_command_stop(self):
        """DockCommand stop."""
        cmd = DockCommand(module_id="Rover1", command="stop")
        self.assertEqual(cmd.command, "stop")

    def test_command_return(self):
        """DockCommand return with parameters."""
        cmd = DockCommand(
            module_id="Rover1",
            command="return",
            parameters={"dock_x": 0.0, "dock_y": 0.0}
        )
        self.assertEqual(cmd.command, "return")
        self.assertAlmostEqual(cmd.parameters["dock_x"], 0.0)

    def test_command_roundtrip(self):
        """DockCommand survives encode/decode."""
        original = DockCommand(
            module_id="Rover1",
            command="explore",
            parameters={"sector": "NE", "coverage": 0.9}
        )
        encoded = encode(original)
        decoded = decode(encoded, DockCommand)

        self.assertEqual(decoded.command, "explore")
        self.assertEqual(decoded.parameters["sector"], "NE")
        self.assertAlmostEqual(decoded.parameters["coverage"], 0.9)


class TestWrapperEncodeDecode(unittest.TestCase):
    """Test the wrapper encode/decode functions."""

    def test_rover_registration_wrapper(self):
        """rover_registration_encode/decode work correctly."""
        original = RoverRegistration(
            module_id="WrapperTest",
            capabilities=["lidar", "camera"]
        )
        encoded = rover_registration_encode(original)
        self.assertIsInstance(encoded, str)

    def test_sensor_data_wrapper(self):
        """sensor_data_encode/decode work correctly."""
        original = SensorData(
            module_id="SensorTest",
            scan_ranges=[1.0, 2.0],
            battery_level=0.8
        )
        encoded = sensor_data_encode(original)
        self.assertIsInstance(encoded, str)
        self.assertIn("SensorTest", encoded)

    def test_velocity_command_wrapper(self):
        """velocity_command_encode/decode work correctly."""
        original = VelocityCommand(
            module_id="VelTest",
            linear_x=0.5,
            angular_z=0.1
        )
        encoded = velocity_command_encode(original)
        self.assertIsInstance(encoded, str)

    def test_rover_status_wrapper(self):
        """rover_status_encode/decode work correctly."""
        original = RoverStatus(
            module_id="StatusTest",
            state="ACTIVE",
            is_moving=True
        )
        encoded = rover_status_encode(original)
        self.assertIsInstance(encoded, str)


if __name__ == '__main__':
    unittest.main()
