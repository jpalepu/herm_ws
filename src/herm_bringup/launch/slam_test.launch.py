import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    SLAM Test Launch - Build a map while driving with Xbox controller.

    Usage:
      ros2 launch herm_bringup slam_test.launch.py

    This launches:
      - Robot state publisher (TF tree from URDF)
      - L298N motor driver (odometry)
      - Xbox controller (teleop)
      - RPLidar (laser scan)
      - SLAM Toolbox (mapping)
      - RViz (visualization)

    Test procedure:
      1. Launch this file
      2. Wait for RViz to open
      3. Drive around slowly with Xbox controller
      4. Watch the map build in real-time
      5. Save map when done: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
    """

    # Package paths
    bringup_pkg = get_package_share_directory('herm_bringup')
    description_pkg = get_package_share_directory('herm_description')

    # URDF
    urdf_file = os.path.join(description_pkg, 'urdf', 'herm.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Config files
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(bringup_pkg, 'config', 'slam.rviz')

    # Launch arguments
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )

    motor_port_arg = DeclareLaunchArgument(
        'motor_port',
        default_value='/dev/ttyCH341USB0',
        description='Motor driver serial port'
    )

    # Robot State Publisher - publishes TF from URDF
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher - for wheel joints
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='log'
    )

    # L298N Motor Driver with calibrated odometry
    l298n_driver = Node(
        package='herm_bringup',
        executable='l298n_driver.py',
        name='l298n_driver',
        output='log',
        parameters=[{
            'serial_port': LaunchConfiguration('motor_port'),
            'baud_rate': 115200,
            'wheel_radius': 0.0335,  # 67mm diameter
            'wheel_separation': 0.34,
            'encoder_cpr': 1320,
            'max_rpm': 330,
            'publish_tf': True,
        }]
    )

    # Joy node for Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    # Xbox teleop
    xbox_teleop = Node(
        package='herm_bringup',
        executable='xbox_test_node.py',
        name='xbox_teleop',
        output='log',
    )

    # RPLidar A2M12
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 256000,  # A2M12 uses 256000
            'frame_id': 'base_scan',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # SLAM Toolbox - Online Async mode for real-time mapping
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        lidar_port_arg,
        motor_port_arg,
        robot_state_pub,
        joint_state_pub,
        l298n_driver,
        joy_node,
        xbox_teleop,
        rplidar_node,
        slam_toolbox,
        rviz_node,
    ])
