"""
hardware_interface.py — Hardware Abstraction Layer for COVEN

!!! DEPRECATED !!!
This file is deprecated and no longer used. Rovers now run Rust firmware
(see lightweight_rover_rs/) instead of Python/ROS2. This file is kept for
reference only and will be removed in a future cleanup.

The Rust implementation has its own hardware abstraction:
- lightweight_rover_rs/src/hardware/motors.rs
- lightweight_rover_rs/src/hardware/encoders.rs
- lightweight_rover_rs/src/hardware/battery.rs

See: lightweight_rover_rs/src/hardware/ for the active hardware drivers.
!!! DEPRECATED !!!

Original description:
Defines abstract interfaces for hardware operations, allowing the same
codebase to work with both simulated and physical hardware.

Author: Alexander Shultis
Date: November 2025
"""

from abc import ABC, abstractmethod
from typing import Optional


class HardwareInterface(ABC):
    """Abstract base class for COVEN hardware operations."""

    @abstractmethod
    def enable_power_rail(self, voltage: int) -> bool:
        """
        Enable power rail to specified voltage.

        Args:
            voltage: Target voltage (5V or 12V)

        Returns:
            True if successful, False otherwise
        """
        pass

    @abstractmethod
    def disable_power_rail(self) -> bool:
        """
        Disable power rail (return to 5V).

        Returns:
            True if successful, False otherwise
        """
        pass

    @abstractmethod
    def detect_ir_beacon(self) -> bool:
        """
        Detect IR beacon signal from docking station.

        Returns:
            True if beacon detected, False otherwise
        """
        pass

    @abstractmethod
    def detect_physical_contact(self) -> bool:
        """
        Detect physical contact with docking pins.

        Returns:
            True if module is physically docked, False otherwise
        """
        pass

    @abstractmethod
    def read_battery_voltage(self) -> float:
        """
        Read current battery voltage.

        Returns:
            Battery voltage in volts
        """
        pass

    @abstractmethod
    def read_battery_current(self) -> float:
        """
        Read current battery current draw.

        Returns:
            Current in amps (positive = charging, negative = discharging)
        """
        pass

    @abstractmethod
    def get_battery_percentage(self) -> float:
        """
        Get estimated battery state of charge.

        Returns:
            Battery percentage (0.0 - 1.0)
        """
        pass

    @abstractmethod
    def control_servo(self, servo_id: int, angle: float) -> bool:
        """
        Control servo motor position.

        Args:
            servo_id: Servo identifier (0-based)
            angle: Target angle in degrees

        Returns:
            True if successful, False otherwise
        """
        pass

    @abstractmethod
    def read_gpio(self, pin: int) -> bool:
        """
        Read digital GPIO pin state.

        Args:
            pin: GPIO pin number

        Returns:
            True if HIGH, False if LOW
        """
        pass

    @abstractmethod
    def write_gpio(self, pin: int, value: bool) -> bool:
        """
        Write digital GPIO pin state.

        Args:
            pin: GPIO pin number
            value: True for HIGH, False for LOW

        Returns:
            True if successful, False otherwise
        """
        pass

    @abstractmethod
    def get_module_id(self) -> Optional[str]:
        """
        Get unique module hardware ID (e.g., from EEPROM or MAC address).

        Returns:
            Module ID string, or None if not available
        """
        pass

    @abstractmethod
    def shutdown(self):
        """Clean up hardware resources."""
        pass
