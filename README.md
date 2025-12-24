# HERM Robot

A 4-wheeled differential drive robot built with ROS2 Humble. This is my personal robotics project running on Jetson Orin Nano.

## What is HERM?

HERM is a skid-steer mobile robot with a layered design (inspired by TurtleBot). It has 4 driven wheels, a 3-tier platform structure, and carries a LiDAR and 360° camera for perception.

### Hardware

- **Compute:** NVIDIA Jetson Orin Nano
- **Motors:** Hiwonder 4-channel encoder motor driver
- **LiDAR:** RPLidar A2M12
- **Camera:** Insta360 Link 2
- **IMU:** MPU3050

### Robot Specs

| Parameter | Value |
|-----------|-------|
| Chassis | 40cm x 30cm x 8cm |
| Wheel Radius | 6.5cm |
| Track Width | 34cm |
| Wheelbase | 30cm |
| Total Height | ~30cm (with all layers) |

## Packages

```
src/
├── herm_description/    # Robot URDF model
├── herm_bringup/        # Hardware drivers and launch files
└── herm_simulation/     # Gazebo simulation (for desktop)
```

### herm_description

Contains the URDF/xacro files describing the robot. The robot has:
- Rectangular base chassis
- 4 wheels (skid-steer configuration)
- 3 stacked layers connected by standoffs
- LiDAR on top layer
- Camera on middle layer
- IMU at base center

### herm_bringup

Drivers and launch files for the real robot:
- Custom camera node using GStreamer (works well on Jetson)
- RPLidar driver configuration
- Motor driver node (Hiwonder serial protocol)
- IMU driver (I2C)

### herm_simulation

Gazebo simulation package for testing on a desktop PC. Includes:
- Gazebo plugins for diff-drive, LiDAR, camera, IMU
- Test world with walls and obstacles
- RViz configuration

## Getting Started

### On Jetson (Real Robot)

```bash
# Clone
cd ~
git clone https://github.com/jpalepu/herm_ws.git
cd herm_ws

# Install dependencies
sudo apt install ros-humble-rplidar-ros python3-opencv ros-humble-cv-bridge
pip3 install pyserial smbus2

# Build
colcon build

# Run sensors test
source install/setup.bash
ros2 launch herm_bringup sensors_test.launch.py
```

### On Desktop (Simulation)

```bash
# Clone
git clone https://github.com/jpalepu/herm_ws.git
cd herm_ws

# Install Gazebo and dependencies
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-teleop-twist-keyboard

# Build
colcon build

# Launch simulation
source install/setup.bash
ros2 launch herm_simulation simulation.launch.py

# In another terminal - control with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Simulation

The simulation package lets you test the robot without hardware. It spawns the robot in a 10x10 meter room with some obstacles.

### Topics in Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |
| `/odom` | Odometry | Robot odometry |
| `/scan` | LaserScan | LiDAR data |
| `/camera/image_raw` | Image | Camera feed |
| `/imu/data` | Imu | IMU readings |

### Launch Options

```bash
# Default launch
ros2 launch herm_simulation simulation.launch.py

# Without RViz
ros2 launch herm_simulation simulation.launch.py use_rviz:=false

# Custom spawn position
ros2 launch herm_simulation simulation.launch.py x_pose:=2.0 y_pose:=1.0
```

## Project Status

- [x] URDF model with 4WD and 3 layers
- [x] LiDAR integration (RPLidar A2M12)
- [x] Camera integration (Insta360 Link 2)
- [x] Gazebo simulation
- [ ] Motor control (Hiwonder driver)
- [ ] IMU integration
- [ ] Teleop with Xbox controller
- [ ] SLAM with slam_toolbox
- [ ] Navigation with Nav2

## Notes

- The simulation uses Gazebo Classic (not Ignition)
- Camera node uses GStreamer backend because standard V4L2 crashes on Jetson with MJPEG cameras
- RPLidar A2M12 runs at 256000 baud rate

## License

Apache 2.0
