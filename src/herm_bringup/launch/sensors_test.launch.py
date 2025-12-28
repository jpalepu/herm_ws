import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package paths
    description_pkg = get_package_share_directory('herm_description')

    # URDF file
    urdf_file = os.path.join(description_pkg, 'urdf', 'herm.urdf.xacro')

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Launch arguments
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device'
    )

    imu_bus_arg = DeclareLaunchArgument(
        'imu_bus',
        default_value='7',
        description='BNO055 I2C bus number'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    return LaunchDescription([
        lidar_port_arg,
        camera_device_arg,
        imu_bus_arg,
        use_rviz_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher (for static joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        # RPLidar A2M12
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 256000,  # A2M12 uses 256000
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'scan_mode': 'Standard'  # Try Standard mode first
            }]
        ),

        # Camera (custom OpenCV+GStreamer node for Jetson)
        Node(
            package='herm_bringup',
            executable='camera_node.py',
            name='camera',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'width': 1280,
                'height': 720,
                'fps': 30,
                'frame_id': 'camera_link',
                'use_gstreamer': True
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ]
        ),

        # BNO055 IMU
        Node(
            package='herm_bringup',
            executable='bno055_imu_node.py',
            name='bno055_imu',
            output='screen',
            parameters=[{
                'i2c_bus': LaunchConfiguration('imu_bus'),
                'i2c_addr': 0x28,
                'frame_id': 'imu_link',
                'publish_rate': 100.0
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('herm_bringup'),
                'config', 'sensors.rviz'
            )],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
