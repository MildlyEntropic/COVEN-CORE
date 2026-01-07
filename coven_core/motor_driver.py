#!/usr/bin/env python3
"""
motor_driver.py - Differential Drive Motor Controller

Subscribes to /cmd_vel (Twist) and outputs PWM signals to TB6612FNG motor driver.

Hardware: TB6612FNG dual H-bridge connected to Raspberry Pi GPIO
- Uses pigpio for hardware-timed PWM (smoother motor control)
- Falls back to gpiod if pigpio unavailable (less smooth)

Author: Alexander Shultis
Date: January 2026
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

# Try to import pigpio for hardware PWM, fall back to gpiod
try:
    import pigpio
    USE_PIGPIO = True
except ImportError:
    USE_PIGPIO = False

try:
    import gpiod
    USE_GPIOD = True
except ImportError:
    USE_GPIOD = False


class MotorDriver(Node):
    """
    Converts Twist messages to differential drive motor commands.

    Subscribes: /{namespace}/cmd_vel (geometry_msgs/Twist)

    Hardware: TB6612FNG dual H-bridge
    - PWMA/PWMB: PWM speed control (0-255 duty cycle)
    - AIN1/AIN2, BIN1/BIN2: Direction control
    - STBY: Standby pin (HIGH to enable)
    """

    # Default GPIO pin assignments (BCM numbering)
    # These can be overridden via parameters
    DEFAULT_PINS = {
        'pwma': 12,   # Left motor PWM (hardware PWM capable)
        'ain1': 5,    # Left motor direction
        'ain2': 6,
        'pwmb': 13,   # Right motor PWM (hardware PWM capable)
        'bin1': 16,   # Right motor direction
        'bin2': 26,
        'stby': 17,   # Standby pin
    }

    def __init__(self):
        super().__init__('motor_driver')

        # Declare parameters
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('wheel_base', 0.1)       # meters between wheels
        self.declare_parameter('wheel_radius', 0.03)    # wheel radius in meters
        self.declare_parameter('max_rpm', 100)          # N20 motor max RPM
        self.declare_parameter('pwm_frequency', 1000)   # PWM frequency Hz
        self.declare_parameter('cmd_timeout', 0.5)      # Stop if no cmd_vel for this long
        self.declare_parameter('invert_left', False)    # Invert left motor direction
        self.declare_parameter('invert_right', False)   # Invert right motor direction

        # GPIO pin parameters (allows rewiring without code changes)
        self.declare_parameter('pin_pwma', self.DEFAULT_PINS['pwma'])
        self.declare_parameter('pin_ain1', self.DEFAULT_PINS['ain1'])
        self.declare_parameter('pin_ain2', self.DEFAULT_PINS['ain2'])
        self.declare_parameter('pin_pwmb', self.DEFAULT_PINS['pwmb'])
        self.declare_parameter('pin_bin1', self.DEFAULT_PINS['bin1'])
        self.declare_parameter('pin_bin2', self.DEFAULT_PINS['bin2'])
        self.declare_parameter('pin_stby', self.DEFAULT_PINS['stby'])

        # Get parameter values
        self.namespace = self.get_parameter('robot_namespace').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value

        # Get pin assignments
        self.pins = {
            'pwma': self.get_parameter('pin_pwma').value,
            'ain1': self.get_parameter('pin_ain1').value,
            'ain2': self.get_parameter('pin_ain2').value,
            'pwmb': self.get_parameter('pin_pwmb').value,
            'bin1': self.get_parameter('pin_bin1').value,
            'bin2': self.get_parameter('pin_bin2').value,
            'stby': self.get_parameter('pin_stby').value,
        }

        # Calculate max wheel velocity (m/s) from max RPM
        self.max_wheel_vel = (self.max_rpm * 2 * 3.14159 * self.wheel_radius) / 60.0

        # Track GPIO initialization state
        self.gpio_initialized = False
        self.pi = None
        self.chip = None
        self.lines = {}

        # Initialize GPIO
        self._init_gpio()

        # Subscribe to cmd_vel
        topic = f'/{self.namespace}/cmd_vel' if self.namespace else '/cmd_vel'
        self.cmd_sub = self.create_subscription(
            Twist, topic, self._cmd_callback, 10
        )

        # Watchdog timer - stop motors if no commands received
        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.1, self._watchdog)

        self.get_logger().info(f'Motor driver initialized, subscribing to {topic}')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m, Max vel: {self.max_wheel_vel:.2f}m/s')

    def _init_gpio(self):
        """Initialize GPIO pins for motor control."""
        if USE_PIGPIO:
            self._init_pigpio()
        elif USE_GPIOD:
            self._init_gpiod()
        else:
            self.get_logger().error('No GPIO library available!')
            self.get_logger().error('Install pigpio: sudo apt install pigpio python3-pigpio')
            self.get_logger().error('Or gpiod: sudo apt install python3-libgpiod')
            return

    def _init_pigpio(self):
        """Initialize using pigpio (preferred - hardware PWM)."""
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Failed to connect to pigpio daemon!')
            self.get_logger().error('Start it with: sudo pigpiod')
            return

        # Set up PWM pins
        self.pi.set_mode(self.pins['pwma'], pigpio.OUTPUT)
        self.pi.set_mode(self.pins['pwmb'], pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pins['pwma'], self.pwm_freq)
        self.pi.set_PWM_frequency(self.pins['pwmb'], self.pwm_freq)

        # Set up direction pins
        for pin_name in ['ain1', 'ain2', 'bin1', 'bin2', 'stby']:
            self.pi.set_mode(self.pins[pin_name], pigpio.OUTPUT)

        # Enable motor driver
        self.pi.write(self.pins['stby'], 1)

        self.gpio_initialized = True
        self.get_logger().info('GPIO initialized via pigpio (hardware PWM)')

    def _init_gpiod(self):
        """Initialize using gpiod (fallback - software PWM)."""
        try:
            self.chip = gpiod.Chip('gpiochip0')
        except Exception as e:
            self.get_logger().error(f'Failed to open GPIO chip: {e}')
            return

        # Set up direction pins (no PWM with gpiod, just on/off)
        for pin_name in ['ain1', 'ain2', 'bin1', 'bin2', 'stby']:
            try:
                line = self.chip.get_line(self.pins[pin_name])
                line.request(consumer='motor_driver', type=gpiod.LINE_REQ_DIR_OUT)
                self.lines[pin_name] = line
            except Exception as e:
                self.get_logger().error(f'Failed to configure GPIO {pin_name}: {e}')
                return

        # Enable standby
        self.lines['stby'].set_value(1)

        self.gpio_initialized = True
        self.get_logger().warn('GPIO initialized via gpiod (no PWM - motors will be on/off only)')

    def _cmd_callback(self, msg: Twist):
        """Convert Twist to differential drive commands."""
        if not self.gpio_initialized:
            return

        self.last_cmd_time = time.time()

        linear = msg.linear.x   # m/s forward
        angular = msg.angular.z  # rad/s counter-clockwise

        # Differential drive kinematics
        # v_left = linear - (angular * wheel_base / 2)
        # v_right = linear + (angular * wheel_base / 2)
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        # Apply inversions if configured
        if self.invert_left:
            v_left = -v_left
        if self.invert_right:
            v_right = -v_right

        # Convert to PWM duty cycle (0-255 for pigpio)
        pwm_left = self._velocity_to_pwm(v_left)
        pwm_right = self._velocity_to_pwm(v_right)

        # Set motor directions and speeds
        self._set_motor('left', pwm_left)
        self._set_motor('right', pwm_right)

    def _velocity_to_pwm(self, velocity: float) -> int:
        """
        Convert wheel velocity (m/s) to PWM value.

        Returns signed value: positive = forward, negative = reverse
        Magnitude is 0-255 for pigpio duty cycle.
        """
        # Clamp to max velocity
        velocity = max(-self.max_wheel_vel, min(self.max_wheel_vel, velocity))

        # Convert to duty cycle (0-255)
        duty = int(abs(velocity) / self.max_wheel_vel * 255)

        # Return signed value (negative = reverse)
        return duty if velocity >= 0 else -duty

    def _set_motor(self, motor: str, pwm: int):
        """
        Set motor direction and speed.

        Args:
            motor: 'left' or 'right'
            pwm: Signed PWM value (-255 to 255)
        """
        if motor == 'left':
            pwm_pin = self.pins['pwma']
            in1, in2 = 'ain1', 'ain2'
        else:
            pwm_pin = self.pins['pwmb']
            in1, in2 = 'bin1', 'bin2'

        if USE_PIGPIO and self.pi:
            if pwm > 0:
                # Forward
                self.pi.write(self.pins[in1], 1)
                self.pi.write(self.pins[in2], 0)
                self.pi.set_PWM_dutycycle(pwm_pin, min(pwm, 255))
            elif pwm < 0:
                # Reverse
                self.pi.write(self.pins[in1], 0)
                self.pi.write(self.pins[in2], 1)
                self.pi.set_PWM_dutycycle(pwm_pin, min(-pwm, 255))
            else:
                # Brake (both HIGH)
                self.pi.write(self.pins[in1], 1)
                self.pi.write(self.pins[in2], 1)
                self.pi.set_PWM_dutycycle(pwm_pin, 0)

        elif USE_GPIOD and self.lines:
            # gpiod fallback (no PWM, just on/off)
            if pwm > 0:
                self.lines[in1].set_value(1)
                self.lines[in2].set_value(0)
            elif pwm < 0:
                self.lines[in1].set_value(0)
                self.lines[in2].set_value(1)
            else:
                # Brake
                self.lines[in1].set_value(1)
                self.lines[in2].set_value(1)

    def _watchdog(self):
        """Stop motors if no commands received recently."""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self._stop_motors()

    def _stop_motors(self):
        """Stop both motors."""
        if self.gpio_initialized:
            self._set_motor('left', 0)
            self._set_motor('right', 0)

    def destroy_node(self):
        """Clean shutdown - stop motors and release GPIO."""
        self.get_logger().info('Shutting down motor driver...')
        self._stop_motors()

        if USE_PIGPIO and self.pi:
            self.pi.write(self.pins['stby'], 0)  # Disable motor driver
            self.pi.stop()

        if USE_GPIOD and self.chip:
            for line in self.lines.values():
                line.release()
            self.chip.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
