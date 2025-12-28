#!/usr/bin/env python3
"""
BNO055 IMU Node for HERM Robot

Reads orientation, angular velocity, and linear acceleration from BNO055
and publishes sensor_msgs/Imu messages.

BNO055 is configured in NDOF mode (9-DOF sensor fusion).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import struct
import math


class BNO055ImuNode(Node):
    # BNO055 I2C Address
    BNO055_ADDR = 0x28

    # Register addresses
    REG_CHIP_ID = 0x00
    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F
    REG_UNIT_SEL = 0x3B
    REG_CALIB_STAT = 0x35

    # Data registers
    REG_ACC_X_LSB = 0x08      # Accelerometer (6 bytes)
    REG_GYR_X_LSB = 0x14      # Gyroscope (6 bytes)
    REG_QUAT_W_LSB = 0x20     # Quaternion (8 bytes)
    REG_LIA_X_LSB = 0x28      # Linear acceleration (6 bytes)

    # Operation modes
    MODE_CONFIG = 0x00
    MODE_NDOF = 0x0C

    # Scale factors
    ACCEL_SCALE = 100.0       # m/s^2 (in NDOF mode with m/s^2 units)
    GYRO_SCALE = 16.0         # rad/s (when configured for radians)
    QUAT_SCALE = (1 << 14)    # Quaternion scale factor

    def __init__(self):
        super().__init__('bno055_imu_node')

        # Parameters
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)

        # Handle string or int for i2c_bus (LaunchConfiguration passes strings)
        bus_param = self.get_parameter('i2c_bus').value
        self.bus_num = int(bus_param) if isinstance(bus_param, str) else bus_param
        self.addr = self.get_parameter('i2c_addr').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('publish_rate').value

        # Publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Initialize I2C
        try:
            self.bus = SMBus(self.bus_num)
            self.get_logger().info(f'Opened I2C bus {self.bus_num}')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus: {e}')
            raise

        # Initialize BNO055
        if not self.init_sensor():
            self.get_logger().error('Failed to initialize BNO055')
            raise RuntimeError('BNO055 initialization failed')

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.rate, self.publish_imu)

        self.get_logger().info(f'BNO055 IMU node started at {self.rate} Hz')

    def init_sensor(self):
        """Initialize BNO055 in NDOF fusion mode."""
        try:
            # Check chip ID
            chip_id = self.bus.read_byte_data(self.addr, self.REG_CHIP_ID)
            if chip_id != 0xA0:
                self.get_logger().error(f'Wrong chip ID: 0x{chip_id:02X} (expected 0xA0)')
                return False
            self.get_logger().info('BNO055 detected')

            # Switch to config mode
            self.bus.write_byte_data(self.addr, self.REG_OPR_MODE, self.MODE_CONFIG)
            self._sleep_ms(25)

            # Reset
            self.bus.write_byte_data(self.addr, self.REG_SYS_TRIGGER, 0x20)
            self._sleep_ms(650)

            # Wait for chip to come back
            while True:
                try:
                    chip_id = self.bus.read_byte_data(self.addr, self.REG_CHIP_ID)
                    if chip_id == 0xA0:
                        break
                except:
                    pass
                self._sleep_ms(10)

            # Normal power mode
            self.bus.write_byte_data(self.addr, self.REG_PWR_MODE, 0x00)
            self._sleep_ms(10)

            # Set units: m/s^2 for accel, rad/s for gyro, radians for euler
            # Bit 1 = 1: Angular rate in rad/s
            # Bit 2 = 0: Euler angles in degrees (we use quaternions anyway)
            # Other bits = 0: defaults
            self.bus.write_byte_data(self.addr, self.REG_UNIT_SEL, 0x02)
            self._sleep_ms(10)

            # Switch to NDOF mode (9-DOF fusion)
            self.bus.write_byte_data(self.addr, self.REG_OPR_MODE, self.MODE_NDOF)
            self._sleep_ms(20)

            self.get_logger().info('BNO055 configured in NDOF mode')
            return True

        except Exception as e:
            self.get_logger().error(f'Init error: {e}')
            return False

    def _sleep_ms(self, ms):
        """Sleep for milliseconds."""
        import time
        time.sleep(ms / 1000.0)

    def read_quaternion(self):
        """Read quaternion (w, x, y, z) from BNO055."""
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.REG_QUAT_W_LSB, 8)
            w = struct.unpack('<h', bytes(data[0:2]))[0] / self.QUAT_SCALE
            x = struct.unpack('<h', bytes(data[2:4]))[0] / self.QUAT_SCALE
            y = struct.unpack('<h', bytes(data[4:6]))[0] / self.QUAT_SCALE
            z = struct.unpack('<h', bytes(data[6:8]))[0] / self.QUAT_SCALE
            return w, x, y, z
        except Exception as e:
            self.get_logger().warn(f'Failed to read quaternion: {e}')
            return 1.0, 0.0, 0.0, 0.0

    def read_gyro(self):
        """Read angular velocity (x, y, z) in rad/s."""
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.REG_GYR_X_LSB, 6)
            x = struct.unpack('<h', bytes(data[0:2]))[0] / self.GYRO_SCALE
            y = struct.unpack('<h', bytes(data[2:4]))[0] / self.GYRO_SCALE
            z = struct.unpack('<h', bytes(data[4:6]))[0] / self.GYRO_SCALE
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f'Failed to read gyro: {e}')
            return 0.0, 0.0, 0.0

    def read_accel(self):
        """Read linear acceleration (x, y, z) in m/s^2."""
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.REG_ACC_X_LSB, 6)
            x = struct.unpack('<h', bytes(data[0:2]))[0] / self.ACCEL_SCALE
            y = struct.unpack('<h', bytes(data[2:4]))[0] / self.ACCEL_SCALE
            z = struct.unpack('<h', bytes(data[4:6]))[0] / self.ACCEL_SCALE
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f'Failed to read accel: {e}')
            return 0.0, 0.0, 0.0

    def get_calibration_status(self):
        """Read calibration status (sys, gyro, accel, mag)."""
        try:
            calib = self.bus.read_byte_data(self.addr, self.REG_CALIB_STAT)
            sys = (calib >> 6) & 0x03
            gyro = (calib >> 4) & 0x03
            accel = (calib >> 2) & 0x03
            mag = calib & 0x03
            return sys, gyro, accel, mag
        except:
            return 0, 0, 0, 0

    def publish_imu(self):
        """Read sensor data and publish Imu message."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Quaternion orientation
        w, x, y, z = self.read_quaternion()
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z

        # Angular velocity (gyroscope)
        gx, gy, gz = self.read_gyro()
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Linear acceleration
        ax, ay, az = self.read_accel()
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Covariance (set to -1 if unknown, or small values for BNO055)
        # BNO055 is quite accurate after calibration
        orientation_cov = 0.0025  # ~0.05 rad std dev
        angular_vel_cov = 0.02   # ~0.14 rad/s std dev
        linear_accel_cov = 0.04  # ~0.2 m/s^2 std dev

        msg.orientation_covariance = [
            orientation_cov, 0.0, 0.0,
            0.0, orientation_cov, 0.0,
            0.0, 0.0, orientation_cov
        ]
        msg.angular_velocity_covariance = [
            angular_vel_cov, 0.0, 0.0,
            0.0, angular_vel_cov, 0.0,
            0.0, 0.0, angular_vel_cov
        ]
        msg.linear_acceleration_covariance = [
            linear_accel_cov, 0.0, 0.0,
            0.0, linear_accel_cov, 0.0,
            0.0, 0.0, linear_accel_cov
        ]

        self.imu_pub.publish(msg)

    def destroy_node(self):
        """Clean up."""
        if hasattr(self, 'bus'):
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = BNO055ImuNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
