import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package path
    pkg_path = get_package_share_directory('herm_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'herm.urdf.xacro')

    # Process xacro and wrap as ParameterValue
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
