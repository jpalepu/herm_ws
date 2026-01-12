from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch L298N motor driver with Xbox controller teleop.

    Usage:
      ros2 launch herm_bringup l298n_xbox.launch.py
      ros2 launch herm_bringup l298n_xbox.launch.py serial_port:=/dev/ttyUSB0
    """

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyCH341USB0',
        description='Arduino serial port'
    )

    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.3',
        description='Max linear velocity (m/s)'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        description='Max angular velocity (rad/s)'
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

    # Teleop node - converts joy to cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 3,
            'scale_linear.x': LaunchConfiguration('max_linear'),
            'scale_angular.yaw': LaunchConfiguration('max_angular'),
            'enable_button': 4,  # LB button as dead-man switch
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
        max_linear_arg,
        max_angular_arg,
        joy_node,
        teleop_node,
        l298n_driver,
    ])
