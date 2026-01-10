#!/usr/bin/env python3
"""
Xbox Controller to Hiwonder Motor Test

A standalone script to test Xbox controller with Hiwonder motor driver.
Reads joystick input and sends motor commands directly via serial.

Usage:
  ros2 run herm_bringup xbox_motor_test.py

Controls:
  - Left Stick Y: Forward/Backward
  - Right Stick X: Turn Left/Right
  - A Button: Emergency stop
  - B Button: Exit
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import math


class XboxMotorTest(Node):
    def __init__(self):
        super().__init__('xbox_motor_test')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear', 0.5)  # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s
        self.declare_parameter('wheel_radius', 0.065)  # m
        self.declare_parameter('wheel_separation', 0.34)  # m
        self.declare_parameter('max_rpm', 330)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_rpm = self.get_parameter('max_rpm').value

        # Serial connection
        self.serial_conn = None
        self.connect_serial()

        # State
        self.enabled = True
        self.last_linear = 0.0
        self.last_angular = 0.0

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Safety timer - stop motors if no input for 500ms
        self.last_joy_time = self.get_clock().now()
        self.create_timer(0.1, self.safety_check)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Xbox Motor Test Started')
        self.get_logger().info(f'Serial: {self.serial_port} @ {self.baud_rate}')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick  = Forward/Backward')
        self.get_logger().info('  Right Stick = Turn Left/Right')
        self.get_logger().info('  A Button    = Emergency Stop')
        self.get_logger().info('  B Button    = Exit')
        self.get_logger().info('=' * 50)

    def connect_serial(self):
        """Connect to motor driver."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.get_logger().warn('Running in simulation mode (no motors)')
            self.serial_conn = None

    def joy_callback(self, msg: Joy):
        """Process joystick input and send motor commands."""
        self.last_joy_time = self.get_clock().now()

        if len(msg.axes) < 4 or len(msg.buttons) < 2:
            return

        # Check buttons
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:  # A button - stop
            self.emergency_stop()
            return

        if len(msg.buttons) > 1 and msg.buttons[1] == 1:  # B button - exit
            self.get_logger().info('Exit requested')
            self.stop_motors()
            rclpy.shutdown()
            return

        if not self.enabled:
            return

        # Read sticks
        linear = msg.axes[1] * self.max_linear   # Left stick Y
        angular = msg.axes[3] * self.max_angular  # Right stick X

        # Only update if values changed significantly
        if abs(linear - self.last_linear) > 0.01 or abs(angular - self.last_angular) > 0.01:
            self.last_linear = linear
            self.last_angular = angular
            self.send_motor_commands(linear, angular)

    def send_motor_commands(self, linear: float, angular: float):
        """Convert velocities to motor commands and send."""
        # Skid-steer kinematics
        v_left = linear - (angular * self.wheel_separation / 2.0)
        v_right = linear + (angular * self.wheel_separation / 2.0)

        # Convert to RPM
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

        # Clamp to max RPM
        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))

        # Send to all 4 motors
        self.set_motor(1, int(rpm_left))   # Front left
        self.set_motor(2, int(rpm_right))  # Front right
        self.set_motor(3, int(rpm_left))   # Rear left
        self.set_motor(4, int(rpm_right))  # Rear right

        # Log output
        left_dir = 'FWD' if rpm_left > 0 else ('REV' if rpm_left < 0 else 'STOP')
        right_dir = 'FWD' if rpm_right > 0 else ('REV' if rpm_right < 0 else 'STOP')
        self.get_logger().info(
            f'L: {rpm_left:+6.1f} RPM ({left_dir}) | R: {rpm_right:+6.1f} RPM ({right_dir})'
        )

    def set_motor(self, motor_id: int, rpm: int):
        """Send command to a single motor."""
        if self.serial_conn is None:
            return

        try:
            direction = 1 if rpm >= 0 else 0
            speed = abs(rpm)

            # Hiwonder protocol
            speed_high = (speed >> 8) & 0xFF
            speed_low = speed & 0xFF
            checksum = (motor_id + direction + speed_high + speed_low) & 0xFF

            cmd = bytes([0xFF, 0xFE, motor_id, direction, speed_high, speed_low, checksum])
            self.serial_conn.write(cmd)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')

    def stop_motors(self):
        """Stop all motors."""
        self.get_logger().info('Stopping all motors')
        for i in range(1, 5):
            self.set_motor(i, 0)
        self.last_linear = 0.0
        self.last_angular = 0.0

    def emergency_stop(self):
        """Emergency stop - disable motors."""
        self.enabled = False
        self.stop_motors()
        self.get_logger().warn('EMERGENCY STOP - Press B to exit, restart to resume')

    def safety_check(self):
        """Stop motors if no joystick input received."""
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > 0.5 and (self.last_linear != 0.0 or self.last_angular != 0.0):
            self.get_logger().warn('No joystick input - stopping motors')
            self.stop_motors()

    def destroy_node(self):
        """Cleanup."""
        self.stop_motors()
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XboxMotorTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
