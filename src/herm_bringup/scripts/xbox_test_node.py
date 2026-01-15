#!/usr/bin/env python3
"""
Simple Xbox Controller to Velocity Test
Directly maps controller input to cmd_vel - no safety button needed.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import sys


class XboxTestNode(Node):
    def __init__(self):
        super().__init__('xbox_test_node')

        # Speed limits
        self.max_linear = 1.0  # m/s
        self.max_angular = 2.0  # rad/s

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('Xbox Test Node Started')
        self.get_logger().info('Left Stick Y = Forward/Back, Right Stick X = Turn')

    def joy_callback(self, msg: Joy):
        if len(msg.axes) < 4:
            self.get_logger().warn('Not enough axes!')
            return

        # Read sticks (axis 1 = left Y, axis 3 = right X)
        linear = msg.axes[1] * self.max_linear  # Forward = positive
        angular = msg.axes[3] * self.max_angular  # Left turn = positive

        # Create and publish cmd_vel
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

        # Log output
        self.get_logger().info(f'Linear: {linear:+.2f} m/s | Angular: {angular:+.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = XboxTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
