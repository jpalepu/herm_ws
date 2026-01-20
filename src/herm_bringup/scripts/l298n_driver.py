#!/usr/bin/env python3
"""
L298N Motor Driver with Encoder Feedback for HERM Robot

Communicates with Arduino Nano running motor_bridge firmware.
- Sends motor commands via serial
- Reads encoder feedback
- Publishes odometry

Usage:
  ros2 run herm_bringup l298n_driver.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import serial
import math
import threading


class L298NDriver(Node):
    def __init__(self):
        super().__init__('l298n_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.0335)  # 67mm diameter
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('encoder_cpr', 1320)  # Counts per revolution
        self.declare_parameter('max_rpm', 330)
        self.declare_parameter('publish_tf', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Serial connection
        self.serial_conn = None
        self.connect_serial()

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_count = 0
        self.last_right_count = 0
        self.last_time = self.get_clock().now()

        # Wheel positions for visualization (radians)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Serial read thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Timer for odometry
        self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info(f'L298N driver started on {self.serial_port}')

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.serial_conn = None

    def cmd_vel_callback(self, msg: Twist):
        if self.serial_conn is None:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert to RPM
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

        # Clamp
        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))

        # Send commands
        self.send_motor_command(1, rpm_left)
        self.send_motor_command(2, rpm_right)

    def send_motor_command(self, motor_id: int, rpm: float):
        if self.serial_conn is None:
            return

        try:
            direction = 1 if rpm >= 0 else 0
            speed = int(abs(rpm))

            speed_high = (speed >> 8) & 0xFF
            speed_low = speed & 0xFF
            checksum = (motor_id + direction + speed_high + speed_low) & 0xFF

            cmd = bytes([0xFF, 0xFE, motor_id, direction, speed_high, speed_low, checksum])
            self.serial_conn.write(cmd)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def read_serial(self):
        while self.running:
            if self.serial_conn is None:
                continue

            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    self.parse_serial_line(line)
            except Exception as e:
                pass

    def parse_serial_line(self, line: str):
        if line.startswith('ENC:'):
            try:
                parts = line[4:].split(',')
                left_count = int(parts[0])
                right_count = int(parts[1])
                self.update_odometry(left_count, right_count)
            except:
                pass

    def update_odometry(self, left_count: int, right_count: int):
        # Calculate deltas
        delta_left = left_count - self.last_left_count
        delta_right = right_count - self.last_right_count
        self.last_left_count = left_count
        self.last_right_count = right_count

        # Convert counts to distance
        dist_left = (delta_left / self.encoder_cpr) * (2 * math.pi * self.wheel_radius)
        dist_right = (delta_right / self.encoder_cpr) * (2 * math.pi * self.wheel_radius)

        # Update wheel positions (radians) for visualization
        self.left_wheel_pos += dist_left / self.wheel_radius
        self.right_wheel_pos += dist_right / self.wheel_radius

        # Calculate robot movement
        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_separation

        # Update pose
        self.x += dist_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += dist_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

    def publish_odometry(self):
        current_time = self.get_clock().now()

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        # TF
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

        # Joint states - actual wheel positions for visualization
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                           'rear_left_wheel_joint', 'rear_right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos,
                               self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = []
        self.joint_pub.publish(joint_state)

    def stop_motors(self):
        self.send_motor_command(1, 0)
        self.send_motor_command(2, 0)

    def destroy_node(self):
        self.running = False
        self.stop_motors()
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = L298NDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
