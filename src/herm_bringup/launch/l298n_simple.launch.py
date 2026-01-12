from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch L298N motor driver with simple Xbox teleop (no dead-man switch).

    Usage:
      ros2 launch herm_bringup l298n_simple.launch.py
      ros2 launch herm_bringup l298n_simple.launch.py serial_port:=/dev/ttyUSB0
    """

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyCH341USB0',
        description='Arduino serial port'
    )

    # Joy node for Xbox controller
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

    # Simple Xbox teleop (our custom node)
    xbox_test = Node(
        package='herm_bringup',
        executable='xbox_test_node.py',
        name='xbox_test_node',
        output='screen',
        parameters=[{
            'max_linear': 0.3,
            'max_angular': 1.0,
        }]
    )

    # L298N motor driver
    l298n_driver = Node(
        package='herm_bringup',
        executable='l298n_driver.py',
        name='l298n_driver',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'wheel_radius': 0.065,
            'wheel_separation': 0.34,
            'encoder_cpr': 1320,
            'max_rpm': 330,
            'publish_tf': True,
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        joy_node,
        xbox_test,
        l298n_driver,
    ])
