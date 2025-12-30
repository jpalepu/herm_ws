import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Xbox controller teleop for the real HERM robot.

    Controls:
      - Left Stick: Forward/Backward
      - Right Stick: Turn Left/Right
      - LB (Left Bumper): Hold to enable driving (safety)
      - RB (Right Bumper): Hold for turbo mode (faster)

    Usage:
      ros2 launch herm_bringup teleop_joy.launch.py
    """
    # Package directories
    bringup_pkg = get_package_share_directory('herm_bringup')

    # Paths
    joy_config = os.path.join(bringup_pkg, 'config', 'xbox_teleop.yaml')

    # Launch configurations
    joy_dev = LaunchConfiguration('joy_dev')

    # Launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    # Joy node - reads joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            joy_config,
            {
                'device_id': 0,
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }
        ]
    )

    # Teleop twist joy - converts joy to cmd_vel
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[joy_config]
    )

    return LaunchDescription([
        # Arguments
        joy_dev_arg,
        # Nodes
        joy_node,
        teleop_joy_node,
    ])
