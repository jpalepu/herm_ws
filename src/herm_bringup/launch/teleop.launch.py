import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package paths
    bringup_pkg = get_package_share_directory('herm_bringup')
    description_pkg = get_package_share_directory('herm_description')

    # URDF file
    urdf_file = os.path.join(description_pkg, 'urdf', 'herm.urdf.xacro')

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Hiwonder motor driver'
    )

    return LaunchDescription([
        serial_port_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Hiwonder Motor Driver
        Node(
            package='herm_bringup',
            executable='hiwonder_driver.py',
            name='hiwonder_driver',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'wheel_radius': 0.065,
                'wheel_separation': 0.34,
                'wheelbase': 0.30,
                'max_rpm': 330,
                'publish_tf': True
            }]
        ),

        # Teleop Twist Keyboard (in separate terminal)
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
            output='screen'
        )
    ])
