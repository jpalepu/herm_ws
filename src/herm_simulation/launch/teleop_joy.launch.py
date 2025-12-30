import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch simulation with Xbox controller teleop.

    Controls:
      - Left Stick: Forward/Backward
      - Right Stick: Turn Left/Right
      - LB (Left Bumper): Hold to enable driving
      - RB (Right Bumper): Hold for turbo mode
    """
    # Package directories
    sim_pkg = get_package_share_directory('herm_simulation')

    # Paths
    joy_config = os.path.join(sim_pkg, 'config', 'xbox_teleop.yaml')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    # Include simulation launch
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
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
        ],
        remappings=[
            ('joy', 'joy'),
        ]
    )

    # Teleop twist joy - converts joy to cmd_vel
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[joy_config],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        joy_dev_arg,
        # Nodes
        simulation,
        joy_node,
        teleop_joy_node,
    ])
