import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Autonomous Navigation Launch - Navigate using a saved map.

    Usage:
      ros2 launch herm_bringup navigation.launch.py map:=/home/jithin/maps/my_home.yaml

    This launches:
      - Robot state publisher (TF tree from URDF)
      - L298N motor driver (odometry + motor control)
      - RPLidar (laser scan)
      - Map server (loads saved map)
      - AMCL (localization)
      - Nav2 stack (planning + control)
      - RViz (visualization + goal setting)

    How to use:
      1. In RViz, click "2D Pose Estimate" and set robot's initial position
      2. Click "2D Goal Pose" to set a navigation goal
      3. Robot will autonomously navigate to the goal
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
    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(bringup_pkg, 'config', 'navigation.rviz')

    # Launch arguments
    default_map_path = os.path.join(os.path.expanduser('~'), 'maps', 'my_home.yaml')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path to map yaml file'
    )

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

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='log'
    )

    # L298N Motor Driver
    l298n_driver = Node(
        package='herm_bringup',
        executable='l298n_driver.py',
        name='l298n_driver',
        output='log',
        parameters=[{
            'serial_port': LaunchConfiguration('motor_port'),
            'baud_rate': 115200,
            'wheel_radius': 0.0335,
            'wheel_separation': 0.34,
            'encoder_cpr': 1320,
            'max_rpm': 330,
            'publish_tf': True,
        }]
    )

    # RPLidar
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 256000,
            'frame_id': 'base_scan',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False
        }]
    )

    # AMCL - Localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', 'cmd_vel')]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # Lifecycle Manager - manages all Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    # RViz - delayed start to allow TF tree to build
    rviz_node = TimerAction(
        period=5.0,  # Wait 5 seconds for TF tree
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config]
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        lidar_port_arg,
        motor_port_arg,
        robot_state_pub,
        joint_state_pub,
        l298n_driver,
        rplidar_node,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        rviz_node,
    ])
