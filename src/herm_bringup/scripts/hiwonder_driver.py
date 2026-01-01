#!/usr/bin/env python3
"""
Hiwonder 4-Channel Encoder Motor Driver for HERM Robot

This node interfaces with the Hiwonder motor driver board via serial.
It subscribes to /cmd_vel and converts twist commands to wheel velocities
for a 4WD skid-steer robot.

Serial Protocol (Hiwonder):
- Baud: 115200
- Command format: 0xFF 0xFE [motor_id] [speed_high] [speed_low] [checksum]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import struct
import math
import time


class HiwonderDriver(Node):
    def __init__(self):
        super().__init__('hiwonder_driver')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_separation', 0.34)  # track width
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('max_rpm', 330)
        self.declare_parameter('encoder_cpr', 1320)  # counts per revolution
        self.declare_parameter('publish_tf', True)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Calculate max speed
        self.max_speed = (self.max_rpm / 60.0) * (2 * math.pi * self.wheel_radius)

        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Encoder counts
        self.encoder_counts = [0, 0, 0, 0]
        self.last_encoder_counts = [0, 0, 0, 0]

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timers
        self.create_timer(0.02, self.read_encoders)  # 50Hz encoder read
        self.create_timer(0.05, self.publish_odometry)  # 20Hz odometry

        self.get_logger().info(f'Hiwonder driver started on {self.serial_port}')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Track: {self.wheel_separation}m')

    def connect_serial(self):
        """Connect to the motor driver via serial."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial: {e}')
            self.serial_conn = None

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist to wheel velocities for 4WD skid-steer."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Skid-steer kinematics
        # Left wheels velocity
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        # Right wheels velocity
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert to RPM
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

        # Clamp to max RPM
        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))

        # Send to motors (front_left, front_right, rear_left, rear_right)
        # Motors 1,3 = left side, Motors 2,4 = right side
        self.set_motor_speed(1, int(rpm_left))   # Front left
        self.set_motor_speed(2, int(rpm_right))  # Front right
        self.set_motor_speed(3, int(rpm_left))   # Rear left
        self.set_motor_speed(4, int(rpm_right))  # Rear right

    def set_motor_speed(self, motor_id: int, rpm: int):
        """Send speed command to a motor."""
        if self.serial_conn is None:
            return

        try:
            # Hiwonder protocol: send motor ID and speed
            # Adjust this based on actual Hiwonder protocol documentation
            direction = 1 if rpm >= 0 else 0
            speed = abs(rpm)

            # Command format (example - adjust for actual protocol)
            # Header: 0xFF 0xFE
            # Motor ID: 1 byte
            # Direction: 1 byte (0=backward, 1=forward)
            # Speed: 2 bytes (high, low)
            # Checksum: 1 byte
            speed_high = (speed >> 8) & 0xFF
            speed_low = speed & 0xFF
            checksum = (motor_id + direction + speed_high + speed_low) & 0xFF

            cmd = bytes([0xFF, 0xFE, motor_id, direction, speed_high, speed_low, checksum])
            self.serial_conn.write(cmd)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def read_encoders(self):
        """Read encoder values from the motor driver."""
        if self.serial_conn is None:
            return

        try:
            # Request encoder data (adjust based on actual protocol)
            # This is a placeholder - implement actual encoder reading
            pass
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')

    def publish_odometry(self):
        """Publish odometry based on wheel velocities."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # For now, publish zero odometry (will be updated with encoder feedback)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Update with encoder
        joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(joint_state)

    def stop_motors(self):
        """Stop all motors."""
        for i in range(1, 5):
            self.set_motor_speed(i, 0)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.stop_motors()
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HiwonderDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
