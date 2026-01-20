from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Odometry Test Launch - Test and calibrate encoder odometry.

    Usage:
      ros2 launch herm_bringup odom_test.launch.py

    This launches:
      - L298N motor driver (sends commands, reads encoders, publishes odom)
      - Xbox controller nodes (for driving)
      - Odometry test display (shows position in real-time)

    Test procedure:
      1. Mark 1 meter on the floor
      2. Drive the robot forward to the mark
      3. Check if X position shows ~1.0 m
      4. Adjust wheel_radius if needed
    """

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0335',  # 67mm diameter / 2
        description='Wheel radius in meters (adjust for calibration)'
    )

    # Joy node
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

    # Xbox teleop (no button needed)
    xbox_node = Node(
        package='herm_bringup',
        executable='xbox_test_node.py',
        name='xbox_test_node',
        output='log',  # Don't clutter terminal
    )

    # L298N driver
    l298n_driver = Node(
        package='herm_bringup',
        executable='l298n_driver.py',
        name='l298n_driver',
        output='log',  # Don't clutter terminal
        parameters=[{
            'serial_port': '/dev/ttyCH341USB0',
            'baud_rate': 115200,
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': 0.34,
            'encoder_cpr': 1320,
            'max_rpm': 330,
            'publish_tf': True,
        }]
    )

    # Odometry test display
    odom_test = Node(
        package='herm_bringup',
        executable='odom_test_node.py',
        name='odom_test_node',
        output='screen',
    )

    return LaunchDescription([
        wheel_radius_arg,
        joy_node,
        xbox_node,
        l298n_driver,
        odom_test,
    ])
