import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package paths
    bringup_pkg = get_package_share_directory('herm_bringup')
    description_pkg = get_package_share_directory('herm_description')

    # Config file
    config_file = os.path.join(bringup_pkg, 'config', 'herm_params.yaml')

    # URDF file
    urdf_file = os.path.join(description_pkg, 'urdf', 'herm.urdf.xacro')

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Launch arguments
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Enable RPLidar'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Enable camera'
    )

    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Enable IMU'
    )

    motor_port_arg = DeclareLaunchArgument(
        'motor_port', default_value='/dev/ttyUSB0',
        description='Motor driver serial port'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB1',
        description='RPLidar serial port'
    )

    return LaunchDescription([
        # Arguments
        use_lidar_arg,
        use_camera_arg,
        use_imu_arg,
        motor_port_arg,
        lidar_port_arg,

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
            parameters=[config_file, {
                'serial_port': LaunchConfiguration('motor_port')
            }]
        ),

        # RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            condition=IfCondition(LaunchConfiguration('use_lidar'))
        ),

        # Camera (v4l2_camera)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link'
            }],
            condition=IfCondition(LaunchConfiguration('use_camera'))
        ),
    ])
