"""
simulated_hardware.py — Simulated Hardware Implementation for COVEN

Provides simulated hardware operations for Gazebo and testing environments.

Author: Alexander Shultis
Date: November 2025
"""

import logging
from typing import Optional
from .hardware_interface import HardwareInterface


logger = logging.getLogger(__name__)


class SimulatedHardware(HardwareInterface):
    """Simulated hardware for Gazebo and testing environments."""

    def __init__(self, module_id: Optional[str] = None):
        """
        Initialize simulated hardware.

        Args:
            module_id: Optional module ID (generated if not provided)
        """
        self.module_id = module_id
        self.power_voltage = 5  # Start at 5V
        self.is_docked = True
        self.battery_voltage = 12.6  # Fully charged Li-ion (3S)
        self.battery_percentage = 1.0
        self.servo_positions = {}
        self.gpio_states = {}

        logger.info("Simulated hardware initialized")

    def enable_power_rail(self, voltage: int) -> bool:
        """Enable power rail to specified voltage."""
        if voltage not in [5, 12]:
            logger.error(f"Invalid voltage: {voltage}V (must be 5V or 12V)")
            return False

        self.power_voltage = voltage
        logger.info(f"Power rail enabled: {voltage}V (simulated)")
        return True

    def disable_power_rail(self) -> bool:
        """Disable power rail (return to 5V)."""
        self.power_voltage = 5
        logger.info("Power rail disabled → 5V (simulated)")
        return True

    def detect_ir_beacon(self) -> bool:
        """Detect IR beacon signal from docking station."""
        # In simulation, always detect beacon when docked
        return self.is_docked

    def detect_physical_contact(self) -> bool:
        """Detect physical contact with docking pins."""
        # In simulation, assume physical contact when docked
        return self.is_docked

    def read_battery_voltage(self) -> float:
        """Read current battery voltage."""
        # Simulate battery discharge (very slow in simulation)
        return self.battery_voltage

    def read_battery_current(self) -> float:
        """Read current battery current draw."""
        # Simulate charging when docked, discharging otherwise
        if self.is_docked:
            return 0.5  # Charging at 500mA
        else:
            return -0.3  # Discharging at 300mA

    def get_battery_percentage(self) -> float:
        """Get estimated battery state of charge."""
        return self.battery_percentage

    def control_servo(self, servo_id: int, angle: float) -> bool:
        """Control servo motor position."""
        if angle < 0 or angle > 180:
            logger.error(f"Invalid servo angle: {angle}° (must be 0-180°)")
            return False

        self.servo_positions[servo_id] = angle
        logger.debug(f"Servo {servo_id} → {angle}° (simulated)")
        return True

    def read_gpio(self, pin: int) -> bool:
        """Read digital GPIO pin state."""
        return self.gpio_states.get(pin, False)

    def write_gpio(self, pin: int, value: bool) -> bool:
        """Write digital GPIO pin state."""
        self.gpio_states[pin] = value
        logger.debug(f"GPIO {pin} → {'HIGH' if value else 'LOW'} (simulated)")
        return True

    def get_module_id(self) -> Optional[str]:
        """Get unique module hardware ID."""
        return self.module_id

    def shutdown(self):
        """Clean up hardware resources."""
        logger.info("Simulated hardware shutdown")
        self.power_voltage = 5
        self.servo_positions.clear()
        self.gpio_states.clear()
