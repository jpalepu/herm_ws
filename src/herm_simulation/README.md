# HERM Simulation

Gazebo Harmonic simulation package for the HERM robot. Use this to test on a desktop PC before deploying to the real robot.

## Requirements

- ROS2 Humble
- Gazebo Harmonic (gz-harmonic)
- ros_gz packages for ROS2-Gazebo bridge
- A machine with decent GPU (simulation uses GPU-accelerated sensors)

## Installation

```bash
# Install Gazebo Harmonic and ROS2 bridge packages
sudo apt install ros-humble-ros-gz ros-humble-teleop-twist-keyboard ros-humble-xacro
```

## Quick Start

```bash
# Terminal 1 - Launch simulation
ros2 launch herm_simulation simulation.launch.py

# Terminal 2 - Drive around
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## What's Simulated

| Sensor/Actuator | Topic | Notes |
|-----------------|-------|-------|
| Differential drive | `/cmd_vel` -> `/odom` | 4WD skid-steer |
| RPLidar | `/scan` | 360deg, 16m range (GPU LiDAR) |
| Camera | `/camera/image_raw` | 1280x720 @ 30fps |
| IMU | `/imu/data` | 100Hz |
| Joint States | `/joint_states` | Wheel positions |

## World

The default world is a 10x10 meter enclosed room with some obstacles (boxes and cylinders). Good for testing navigation and obstacle avoidance.

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | true | Use Gazebo clock |
| `use_rviz` | true | Launch RViz |
| `world` | herm_world.sdf | World file path |
| `x_pose` | 0.0 | Spawn X position |
| `y_pose` | 0.0 | Spawn Y position |

## ROS-Gazebo Bridge

The simulation uses `ros_gz_bridge` to translate between Gazebo transport topics and ROS2 topics:

- Clock synchronization (`/clock`)
- Sensor data (LiDAR, camera, IMU)
- Control commands (`/cmd_vel`)
- Odometry and TF

## Troubleshooting

**Gazebo fails to start**
- Make sure Gazebo Harmonic is installed: `gz sim --version`
- Check GPU drivers are working: `glxinfo | grep "OpenGL"`

**Robot falls through floor**
- Wait a few seconds before sending commands
- The physics engine needs time to stabilize

**LiDAR not showing in RViz**
- Check that Fixed Frame is set to `odom` or `base_footprint`
- Make sure `/scan` topic is being published: `ros2 topic hz /scan`

**Topics not appearing in ROS2**
- Check the bridge is running: `ros2 node list | grep bridge`
- Verify Gazebo topics: `gz topic -l`

## File Structure

```
herm_simulation/
├── urdf/
│   └── herm_gazebo.urdf.xacro  # Robot + Gazebo Harmonic plugins
├── worlds/
│   └── herm_world.sdf          # Test environment (SDF format)
├── config/
│   └── simulation.rviz         # RViz layout
└── launch/
    └── simulation.launch.py    # Main launch
```
