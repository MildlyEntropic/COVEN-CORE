"""
simulated_hardware.py — Simulated Hardware Implementation for COVEN

!!! DEPRECATED !!!
This file is deprecated and no longer used. Rovers now run Rust firmware
(see lightweight_rover_rs/) instead of Python/ROS2. This file is kept for
reference only and will be removed in a future cleanup.

For simulation/testing, the Rust firmware has:
- lightweight_rover_rs/src/mock.rs (mock hardware for testing)
- Run with: coven-rover --mock

See: lightweight_rover_rs/src/mock.rs for the active mock implementation.
!!! DEPRECATED !!!

Original description:
Provides simulated hardware operations for Gazebo and testing environments.

Author: Alexander Shultis
Date: November 2025
"""

import logging
import random
from typing import Optional
from .hardware_interface import HardwareInterface


logger = logging.getLogger(__name__)


class SimulatedHardware(HardwareInterface):
    """Simulated hardware for Gazebo and testing environments."""

    def __init__(self, module_id: Optional[str] = None, randomize_battery: bool = True):
        """
        Initialize simulated hardware.

        Args:
            module_id: Optional module ID (generated if not provided)
            randomize_battery: If True, start with random battery (60-100%)
        """
        self.module_id = module_id
        self.power_voltage = 5  # Start at 5V
        self.is_docked = True
        self.servo_positions = {}
        self.gpio_states = {}

        # Randomize starting battery for realistic bidding simulation
        if randomize_battery:
            self.battery_percentage = random.uniform(0.6, 1.0)
        else:
            self.battery_percentage = 1.0

        # Battery voltage scales with percentage (3S Li-ion: 9.6V empty, 12.6V full)
        self.battery_voltage = 9.6 + (self.battery_percentage * 3.0)

        logger.info(f"Simulated hardware initialized (battery: {self.battery_percentage*100:.0f}%)")

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

    def drain_battery(self, distance_meters: float) -> None:
        """
        Simulate battery drain based on distance traveled.

        Args:
            distance_meters: Distance traveled in meters

        Drain rate: ~2% per 10 meters (rough estimate for small rover)
        """
        drain = distance_meters * 0.002  # 0.2% per meter
        self.battery_percentage = max(0.1, self.battery_percentage - drain)
        self.battery_voltage = 9.6 + (self.battery_percentage * 3.0)
        logger.debug(f"Battery drained {drain*100:.1f}% → {self.battery_percentage*100:.0f}%")

    def charge_battery(self, amount: float = 0.1) -> None:
        """
        Simulate battery charging while docked.

        Args:
            amount: Amount to charge (0.0-1.0), default 10%
        """
        self.battery_percentage = min(1.0, self.battery_percentage + amount)
        self.battery_voltage = 9.6 + (self.battery_percentage * 3.0)
        logger.debug(f"Battery charged → {self.battery_percentage*100:.0f}%")

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
