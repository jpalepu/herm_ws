#!/usr/bin/env python3
"""
Odometry Test Node for HERM Robot

Displays real-time odometry data for calibration and testing.
Run this alongside l298n_driver to verify encoder accuracy.

Test procedure:
1. Place robot at a known starting point
2. Mark 1 meter on the floor
3. Drive forward to the mark
4. Check if X position shows ~1.0m
5. Adjust wheel_radius parameter if needed
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class OdomTestNode(Node):
    def __init__(self):
        super().__init__('odom_test_node')

        # Track odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Timer to print status
        self.create_timer(0.5, self.print_status)

        self.get_logger().info('=' * 50)
        self.get_logger().info('ODOMETRY TEST NODE')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Instructions:')
        self.get_logger().info('1. Mark 1 meter on the floor')
        self.get_logger().info('2. Drive robot forward to the mark')
        self.get_logger().info('3. X should show ~1.0 m')
        self.get_logger().info('4. Press Ctrl+C to reset and restart')
        self.get_logger().info('=' * 50)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Calculate theta from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        # Calculate total distance traveled
        dx = self.x - self.last_x
        dy = self.y - self.last_y
        self.total_distance += math.sqrt(dx * dx + dy * dy)
        self.last_x = self.x
        self.last_y = self.y

    def print_status(self):
        theta_deg = math.degrees(self.theta)

        print('\033[2J\033[H')  # Clear screen
        print('=' * 50)
        print('        ODOMETRY TEST - LIVE DATA')
        print('=' * 50)
        print(f'  X Position:     {self.x:+.3f} m')
        print(f'  Y Position:     {self.y:+.3f} m')
        print(f'  Heading:        {theta_deg:+.1f} deg')
        print('-' * 50)
        print(f'  Total Distance: {self.total_distance:.3f} m')
        print('=' * 50)
        print('')
        print('CALIBRATION TEST:')
        print('  1. Drive forward exactly 1 meter')
        print('  2. X should show ~1.000 m')
        print('')
        print('  If X shows 0.8m after 1m drive:')
        print('    -> Increase wheel_radius')
        print('  If X shows 1.2m after 1m drive:')
        print('    -> Decrease wheel_radius')
        print('')
        print('  Current wheel_radius: 0.0335 m (67mm diameter)')
        print('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
