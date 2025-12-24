# HERM Simulation

Gazebo simulation package for the HERM robot. Use this to test on a desktop PC before deploying to the real robot.

## Requirements

- ROS2 Humble
- Gazebo Classic (comes with ros-humble-gazebo-ros-pkgs)
- A machine with decent GPU (simulation is heavy)

## Installation

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-teleop-twist-keyboard ros-humble-xacro
```

## Quick Start

```bash
# Terminal 1 - Launch simulation
ros2 launch herm_simulation simulation.launch.py

# Terminal 2 - Drive around
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or use the combined launch:
```bash
ros2 launch herm_simulation teleop_sim.launch.py
```

## What's Simulated

| Sensor/Actuator | Topic | Notes |
|-----------------|-------|-------|
| Differential drive | `/cmd_vel` → `/odom` | 4WD skid-steer |
| RPLidar | `/scan` | 360°, 16m range |
| Camera | `/camera/image_raw` | 1280x720 @ 30fps |
| IMU | `/imu/data` | 100Hz |

## World

The default world is a 10x10 meter enclosed room with some obstacles (boxes and cylinders). Good for testing navigation and obstacle avoidance.

To use an empty world:
```bash
ros2 launch herm_simulation simulation.launch.py world:=/usr/share/gazebo-11/worlds/empty.world
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | true | Use Gazebo clock |
| `use_rviz` | true | Launch RViz |
| `world` | herm_world.world | World file path |
| `x_pose` | 0.0 | Spawn X position |
| `y_pose` | 0.0 | Spawn Y position |

## Troubleshooting

**Gazebo crashes on startup**
- Make sure you have enough GPU memory
- Try running `gazebo --verbose` to see errors

**Robot falls through floor**
- Wait a few seconds before sending commands
- The physics engine needs time to stabilize

**LiDAR not showing in RViz**
- Check that Fixed Frame is set to `odom` or `base_footprint`
- Make sure `/scan` topic is being published: `ros2 topic hz /scan`

## File Structure

```
herm_simulation/
├── urdf/
│   └── herm_gazebo.urdf.xacro  # Robot + Gazebo plugins
├── worlds/
│   └── herm_world.world        # Test environment
├── config/
│   └── simulation.rviz         # RViz layout
└── launch/
    ├── simulation.launch.py    # Main launch
    └── teleop_sim.launch.py    # With keyboard control
```
