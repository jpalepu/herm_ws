from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Simple Xbox controller test - no safety button required."""

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

    xbox_test_node = Node(
        package='herm_bringup',
        executable='xbox_test_node.py',
        name='xbox_test_node',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        xbox_test_node,
    ])
