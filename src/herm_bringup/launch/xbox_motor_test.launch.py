from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Xbox controller with motor driver test.

    Usage:
      ros2 launch herm_bringup xbox_motor_test.launch.py
      ros2 launch herm_bringup xbox_motor_test.launch.py serial_port:=/dev/ttyUSB1
    """

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Motor driver serial port'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    motor_test_node = Node(
        package='herm_bringup',
        executable='xbox_motor_test.py',
        name='xbox_motor_test',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'max_linear': 0.3,   # Conservative speed for testing
            'max_angular': 0.5,
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        joy_node,
        motor_test_node,
    ])
