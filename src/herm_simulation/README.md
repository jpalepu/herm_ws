# HERM Simulation

Gazebo Harmonic simulation package for the HERM robot with SLAM and autonomous navigation support.

## Requirements

- ROS2 Humble
- Gazebo Harmonic (gz-harmonic)
- ros_gz packages for ROS2-Gazebo bridge
- Nav2 navigation stack
- SLAM Toolbox
- A machine with decent GPU (simulation uses GPU-accelerated sensors)

## Installation

```bash
# Install Gazebo Harmonic, Nav2, and SLAM packages
sudo apt install ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-xacro
```

## Quick Start

### Basic Simulation (Manual Control)

```bash
# Terminal 1 - Launch simulation
ros2 launch herm_simulation simulation.launch.py

# Terminal 2 - Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or use Xbox controller
ros2 launch herm_simulation teleop_joy.launch.py
```

### Xbox Controller

```bash
# Install joystick packages
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

# Launch simulation with Xbox controller
ros2 launch herm_simulation teleop_joy.launch.py
```

**Controls:**
- **Left Stick**: Forward/Backward
- **Right Stick**: Turn Left/Right
- **LB (Left Bumper)**: Hold to enable driving (safety)
- **RB (Right Bumper)**: Hold for turbo mode (faster)

### SLAM (Build a Map)

```bash
# Launch simulation with SLAM
ros2 launch herm_simulation slam.launch.py

# In another terminal, drive around to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/herm_ws/src/herm_simulation/maps/my_map
```

### Navigation (Use Saved Map)

```bash
# Launch with your saved map
ros2 launch herm_simulation navigation.launch.py map:=/path/to/my_map.yaml

# Set initial pose in RViz (2D Pose Estimate button)
# Click Nav2 Goal to send the robot to a destination
```

### SLAM + Navigation (Explore and Navigate)

```bash
# Launch SLAM with Nav2 - navigate while building map
ros2 launch herm_simulation slam_navigation.launch.py

# Drive manually first to build initial map, then use Nav2 goals
```

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `simulation.launch.py` | Basic simulation with Gazebo + RViz |
| `teleop_joy.launch.py` | Simulation + Xbox controller teleop |
| `slam.launch.py` | Simulation + SLAM Toolbox for mapping |
| `navigation.launch.py` | Simulation + Nav2 with saved map |
| `slam_navigation.launch.py` | Simulation + SLAM + Nav2 for autonomous exploration |

## What's Simulated

| Sensor/Actuator | Topic | Notes |
|-----------------|-------|-------|
| Differential drive | `/cmd_vel` -> `/odom` | 4WD skid-steer |
| RPLidar | `/scan` | 360deg, 16m range (GPU LiDAR) |
| Camera | `/camera/image_raw` | 1280x720 @ 30fps |
| IMU | `/imu/data` | 100Hz |

## Nav2 Features

- **Path Planning:** NavFn global planner
- **Local Control:** DWB local planner with dynamic obstacle avoidance
- **Recovery Behaviors:** Spin, backup, wait
- **Costmaps:** Global and local costmaps with inflation
- **Behavior Trees:** Full Nav2 BT navigation

## Configuration Files

| File | Description |
|------|-------------|
| `config/nav2_params.yaml` | Nav2 stack configuration |
| `config/slam_toolbox_params.yaml` | SLAM Toolbox settings |
| `config/slam.rviz` | RViz config for mapping |
| `config/navigation.rviz` | RViz config for navigation |

## Setting Navigation Goals

In RViz:
1. Use **2D Pose Estimate** tool to set initial robot position (if using AMCL)
2. Use **Nav2 Goal** tool to click a destination
3. Robot will plan path and navigate autonomously

## Saving Maps

After building a map with SLAM:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/herm_ws/src/herm_simulation/maps/my_map
```

This creates `my_map.yaml` and `my_map.pgm` files.

## Troubleshooting

**Nav2 nodes not starting**
- Check lifecycle manager: `ros2 lifecycle list /lifecycle_manager_navigation`
- Nodes need to be in "active" state

**Robot not moving to goal**
- Ensure `/scan` topic is publishing: `ros2 topic hz /scan`
- Check costmaps are being generated: `ros2 topic echo /global_costmap/costmap`

**SLAM map looks bad**
- Drive slower for better scan matching
- Ensure good loop closures by returning to starting area

**"Transform not available" errors**
- Wait for TF tree to be fully published
- Check frames: `ros2 run tf2_tools view_frames`

## File Structure

```
herm_simulation/
├── config/
│   ├── nav2_params.yaml         # Nav2 configuration
│   ├── slam_toolbox_params.yaml # SLAM settings
│   ├── xbox_teleop.yaml         # Xbox controller config
│   ├── slam.rviz                # RViz for mapping
│   ├── navigation.rviz          # RViz for navigation
│   └── simulation.rviz          # Basic RViz config
├── launch/
│   ├── simulation.launch.py     # Basic simulation
│   ├── teleop_joy.launch.py     # Xbox controller teleop
│   ├── slam.launch.py           # SLAM mapping
│   ├── navigation.launch.py     # Nav2 with map
│   └── slam_navigation.launch.py # SLAM + Nav2
├── maps/                        # Saved maps directory
├── urdf/
│   └── herm_gazebo.urdf.xacro   # Robot + Gazebo plugins
└── worlds/
    └── herm_world.sdf           # Test environment
```
