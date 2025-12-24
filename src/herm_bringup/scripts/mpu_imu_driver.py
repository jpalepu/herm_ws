#!/usr/bin/env python3
"""
MPU3050/MPU6050 IMU Driver for HERM Robot

Reads gyroscope (and accelerometer if MPU6050) data via I2C
and publishes to /imu/data topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time


class MPUImuDriver(Node):
    # MPU registers
    PWR_MGMT_1 = 0x6B
    GYRO_XOUT_H = 0x43
    ACCEL_XOUT_H = 0x3B
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    # Scale factors
    GYRO_SCALE = 131.0  # for ±250°/s
    ACCEL_SCALE = 16384.0  # for ±2g

    def __init__(self):
        super().__init__('mpu_imu_driver')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('use_accelerometer', True)

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_accel = self.get_parameter('use_accelerometer').value

        # Initialize I2C
        self.bus = None
        self.init_imu()

        # Publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Timer
        period = 1.0 / self.publish_rate
        self.create_timer(period, self.publish_imu)

        self.get_logger().info(f'MPU IMU driver started on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02X}')

    def init_imu(self):
        """Initialize the MPU sensor."""
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)

            # Wake up the MPU (clear sleep bit)
            self.bus.write_byte_data(self.i2c_address, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)

            # Configure gyroscope (±250°/s)
            self.bus.write_byte_data(self.i2c_address, self.GYRO_CONFIG, 0x00)

            # Configure accelerometer (±2g) if available
            if self.use_accel:
                try:
                    self.bus.write_byte_data(self.i2c_address, self.ACCEL_CONFIG, 0x00)
                except Exception:
                    self.use_accel = False
                    self.get_logger().warn('Accelerometer not available (MPU3050?)')

            self.get_logger().info('MPU initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU: {e}')
            self.bus = None

    def read_word(self, reg):
        """Read a 16-bit signed value from two registers."""
        high = self.bus.read_byte_data(self.i2c_address, reg)
        low = self.bus.read_byte_data(self.i2c_address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def publish_imu(self):
        """Read IMU data and publish."""
        if self.bus is None:
            return

        try:
            # Read gyroscope
            gyro_x = self.read_word(self.GYRO_XOUT_H) / self.GYRO_SCALE
            gyro_y = self.read_word(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE
            gyro_z = self.read_word(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE

            # Convert to rad/s
            gyro_x = math.radians(gyro_x)
            gyro_y = math.radians(gyro_y)
            gyro_z = math.radians(gyro_z)

            # Read accelerometer if available
            if self.use_accel:
                accel_x = self.read_word(self.ACCEL_XOUT_H) / self.ACCEL_SCALE * 9.81
                accel_y = self.read_word(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE * 9.81
                accel_z = self.read_word(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE * 9.81
            else:
                accel_x = accel_y = accel_z = 0.0

            # Create IMU message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Orientation not provided by raw IMU
            msg.orientation_covariance[0] = -1

            # Angular velocity
            msg.angular_velocity.x = gyro_x
            msg.angular_velocity.y = gyro_y
            msg.angular_velocity.z = gyro_z
            msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            # Linear acceleration
            msg.linear_acceleration.x = accel_x
            msg.linear_acceleration.y = accel_y
            msg.linear_acceleration.z = accel_z
            msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]

            self.imu_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}')

    def destroy_node(self):
        """Clean up."""
        if self.bus:
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPUImuDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
