import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directory
    sim_pkg = get_package_share_directory('herm_simulation')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    return LaunchDescription([
        use_rviz_arg,

        # Include main simulation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz')
            }.items()
        ),

        # Teleop keyboard (in new terminal)
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
            output='screen'
        )
    ])
