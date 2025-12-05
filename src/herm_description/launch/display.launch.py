from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    pkg_path = os.path.join(
        os.getenv("HOME"),
        "herm_ws/src/herm_description"
    )
    urdf_file = os.path.join(pkg_path, "urdf/herm.urdf.xacro")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_file])
            }]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),

        Node(
            package="rviz2",
            executable="rviz2"
        )
    ])
