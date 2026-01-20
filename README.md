# HERM

**Human-Experience Robotic Machine**

An open-source robotics project focused on improving human-agent collaboration and building better robotic systems that work alongside people.

---

## About This Project

HERM started as an idea to build a robot that doesn't just perform tasks, but actually collaborates with humans in a meaningful way. The goal isn't to create another robot that follows commands, it's to explore how robots and AI agents can work together with people, understand context, and become genuine partners in real-world environments.

This is an open-source effort. All development, updates, experiments, and learnings will be shared here. Whether you're into robotics, AI, ROS2, or just curious about where this kind of tech is heading, you're welcome to follow along, contribute, or fork it for your own experiments.

## Why HERM?

Most robots today are either industrial machines locked behind cages or simple toys with limited capability. There's a gap in the middle — robots that can operate in human spaces, understand what's happening around them, and actually be helpful without needing an engineering degree to operate.

HERM is an attempt to bridge that gap. It's built on accessible hardware, runs on ROS2, and is designed to be a platform for experimenting with:

- **Multi-agent collaboration** — robots working with AI systems and humans
- **Context-aware behavior** — understanding environments, not just mapping them
- **Natural interaction** — making robots that people actually want to work with

---

## The Robot

HERM is a 4-wheeled skid-steer mobile robot with a layered platform design. It's compact enough to navigate indoor spaces but capable enough to carry sensors and compute for real autonomy. (future dev version will be released in Q1 2026 so wait for it)

### Hardware Stack (currently)

| Component | What We're Using |
|-----------|------------------|
| Compute | NVIDIA Jetson Orin Nano |
| Motor Driver | L298N + Arduino Nano (serial bridge) |
| Motors | 25GA-371 DC motors with encoders |
| LiDAR | RPLidar A2M12 (360°, 16m range) |
| Camera | Insta360 Link 2 (webcam, 1280x720) |
| IMU | BNO055 (9-DOF with onboard fusion) |

### Physical Specs

| Spec | Value |
|------|-------|
| Chassis | 40 x 30 x 8 cm |
| Wheel Radius | 3.35 cm (67mm diameter) |
| Track Width | 34 cm |
| Wheelbase | 30 cm |
| Total Height | ~30 cm |
| Drive | 4WD skid-steer |

The design takes inspiration from TurtleBot's layered approach — three platforms stacked with standoffs, giving room for electronics, sensors, and future expansion.

---

## Repository Structure

```
src/
├── herm_description/    # Robot URDF model
├── herm_bringup/        # Hardware drivers and launch files
└── herm_simulation/     # Gazebo simulation (for desktop testing)
```

| Package | Purpose |
|---------|---------|
| `herm_description` | URDF/xacro model of the robot — the physical description that both real hardware and simulation use |
| `herm_bringup` | Everything needed to run the real robot: sensor drivers, motor control, launch files |
| `herm_simulation` | Gazebo simulation for testing without hardware — great for development on any PC |

---

## Getting Started

### Prerequisites

- ROS2 Humble
- Ubuntu 22.04 (tested on both x86 and ARM/Jetson)

### Running on Real Hardware (Jetson)

```bash
git clone https://github.com/jpalepu/herm_ws.git
cd herm_ws

# Install dependencies
sudo apt install ros-humble-rplidar-ros python3-opencv ros-humble-cv-bridge
pip3 install pyserial smbus2

# Build
colcon build
source install/setup.bash

# Test sensors
ros2 launch herm_bringup sensors_test.launch.py
```

### Running in Simulation (Desktop)

Don't have the hardware? No problem. The simulation package lets you test everything on your laptop or desktop using Gazebo Harmonic.

```bash
git clone https://github.com/jpalepu/herm_ws.git
cd herm_ws

# Install Gazebo Harmonic and ROS2 bridge packages
sudo apt install ros-humble-ros-gz ros-humble-teleop-twist-keyboard

# Build
colcon build
source install/setup.bash

# Launch simulation
ros2 launch herm_simulation simulation.launch.py
```

Then in another terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use WASD keys to drive around. The simulated world has some walls and obstacles to play with.

---

## Current Status

This is an active project. Here's where things stand:

| Feature | Status |
|---------|--------|
| Robot URDF (4WD, 3-layer design) | Done |
| LiDAR integration (RPLidar A2M12) | Done |
| Camera integration (GStreamer) | Done |
| IMU integration (BNO055) | Done |
| Gazebo Harmonic simulation | Done |
| SLAM (slam_toolbox) | Done |
| Nav2 autonomous navigation | Done |
| Teleop (keyboard + Xbox) | Done |
| Motor control (L298N + encoders) | Done |
| Agent collaboration framework | Planned |

---

## Roadmap

The bigger picture for HERM goes beyond just building a robot that moves around. Here's where this is heading:

**Phase 1 — Foundation (current)**
- Basic mobility and sensor integration
- Simulation environment for testing
- Manual control via teleop

**Phase 2 — Autonomy**
- SLAM and mapping
- Autonomous navigation with Nav2
- Obstacle avoidance and path planning

**Phase 3 — Intelligence**
- Integration with AI agents
- Context understanding and scene awareness
- Natural language interaction

**Phase 4 — Collaboration**
- Multi-robot coordination
- Human-robot teaming experiments
- Shared task execution

---

## Contributing

This is an open project and contributions are welcome. Whether it's fixing a bug, improving documentation, adding features, or just sharing ideas — all input helps.

If you're interested in contributing:
1. Fork the repo
2. Create a branch for your changes
3. Submit a pull request

Or just open an issue if you have questions or suggestions.

---

## Sensor Setup

### BNO055 IMU

The BNO055 is a 9-DOF IMU with onboard sensor fusion. It outputs quaternion orientation directly, no external filtering needed.

**Connection:** I2C (default address 0x28)

```bash
# Install dependency
pip3 install smbus2

# Test IMU
ros2 run herm_bringup bno055_imu_node.py --ros-args -p i2c_bus:=7
```

**Topics:**
- `/imu/data` (sensor_msgs/Imu) — orientation, angular velocity, linear acceleration at 100Hz

**Mounting:** Orient the BNO055 so its X-axis points forward on the robot (follows REP-103).

### RPLidar A2M12

360-degree laser scanner with 16m range.

**Connection:** USB serial (typically `/dev/ttyUSB0`)

```bash
# Give permission
sudo chmod 666 /dev/ttyUSB0

# Test LiDAR
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000
```

**Topics:**
- `/scan` (sensor_msgs/LaserScan) — 360 samples at 10Hz

### Camera (Insta360 Link 2)

Uses GStreamer pipeline because standard V4L2 doesn't handle the MJPEG stream well on Jetson.

```bash
# Test camera
ros2 run herm_bringup camera_node.py --ros-args -p device:=/dev/video0
```

**Topics:**
- `/camera/image_raw` (sensor_msgs/Image) — 1280x720 at 30fps
- `/camera/camera_info` (sensor_msgs/CameraInfo)

### Xbox Controller

#### Bluetooth Setup (Jetson Orin Nano)

```bash
# Install required packages
sudo apt install joystick jstest-gtk xboxdrv

# Disable Bluetooth ERTM (required for Xbox controllers)
sudo bash -c 'echo 1 > /sys/module/bluetooth/parameters/disable_ertm'

# Make permanent
sudo bash -c 'echo "options bluetooth disable_ertm=1" >> /etc/modprobe.d/bluetooth.conf'

# Pair the controller
bluetoothctl
# In bluetoothctl:
scan on
# Put controller in pairing mode (Xbox + sync button)
pair <MAC_ADDRESS>
trust <MAC_ADDRESS>
connect <MAC_ADDRESS>
exit
```

**Important:** Update the Xbox controller firmware using a Windows PC or Xbox console for stable Bluetooth connection.

#### Testing the Controller

```bash
# Install ROS2 joystick packages
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

# Test raw joystick input
jstest /dev/input/js0

# Test with ROS2 (simple velocity output)
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup xbox_test.launch.py
```

The test node shows real-time velocity output:
```
[xbox_test_node]: Linear: +0.35 m/s | Angular: +0.00 rad/s
```

#### Teleop Launch Files

```bash
# Real robot with safety button (LB to enable)
ros2 launch herm_bringup teleop_joy.launch.py

# Simulation with Xbox controller
ros2 launch herm_simulation teleop_joy.launch.py

# Simple test (no safety button)
ros2 launch herm_bringup xbox_test.launch.py
```

#### Controls

| Input | Action |
|-------|--------|
| Left Stick Y | Forward/Backward |
| Right Stick X | Turn Left/Right |
| LB (hold) | Enable driving (safety) |
| RB (hold) | Turbo mode (faster) |

#### Troubleshooting

- **LED keeps blinking:** Controller not fully paired. Run `bluetoothctl trust <MAC>` and reconnect
- **No input detected:** Check `ros2 topic echo /joy` to verify data is being published
- **Controller disconnects:** Update controller firmware and ensure ERTM is disabled

### L298N Motor Driver

The robot uses an L298N motor driver controlled via Arduino Nano over serial.

**Wiring:**

| L298N | Arduino Nano |
|-------|--------------|
| ENA | D5 |
| IN1 | D2 |
| IN2 | D3 |
| ENB | D6 |
| IN3 | D4 |
| IN4 | D7 |
| GND | GND |

| Motor Encoder | Arduino Nano |
|---------------|--------------|
| Left Encoder A | D8 |
| Left Encoder B | D9 |
| Right Encoder A | D10 |
| Right Encoder B | D11 |
| Encoder 3.3V | 3.3V |
| Encoder GND | GND |

**Power:** Connect 11.1V LiPo to L298N +12V and GND terminals.

**Arduino Firmware:** Upload the firmware from `~/arduino/motor_bridge/motor_bridge.ino`

```bash
arduino-cli upload -p /dev/ttyCH341USB0 -b arduino:avr:nano ~/arduino/motor_bridge/
```

**Testing Motors with Xbox Controller:**

```bash
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup l298n_simple.launch.py
```

Controls (no buttons needed):
- Left stick up/down: Forward/Backward
- Right stick left/right: Turn

---

## SLAM and Autonomous Navigation

This section covers mapping your environment and setting up autonomous navigation with Nav2.

### Prerequisites - Install Nav2

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

### Step 1: Build a Map (SLAM)

Before autonomous navigation, you need a map of your environment.

**Terminal 1 - Launch SLAM (keep running):**
```bash
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup slam_test.launch.py
```

This launches:
- Motor driver with odometry
- Xbox controller for manual driving
- RPLidar for laser scans
- SLAM Toolbox for mapping
- RViz for visualization

**How to map:**
1. Wait for RViz to open
2. Drive slowly around your space using Xbox controller (left stick)
3. Watch the map build in real-time (gray = explored, black = walls)
4. Cover the entire area you want the robot to navigate

**Terminal 2 - Save the map (when done exploring):**
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_home
```

This creates:
- `~/maps/my_home.pgm` - The map image
- `~/maps/my_home.yaml` - Map metadata

**Important:** Keep Terminal 1 running while saving. Don't close SLAM until the map is saved.

---

### Step 2: Autonomous Navigation (Nav2)

Once you have a map, the robot can navigate autonomously.

**Terminal 1 - Launch Navigation:**
```bash
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py map:=/home/jithin/maps/my_home.yaml
```

This launches:
- Motor driver (for movement)
- RPLidar (for obstacle detection)
- Map server (loads your saved map)
- AMCL (localization - figures out where robot is)
- Nav2 stack (path planning + control)
- RViz (visualization + goal setting)

**How to navigate:**

| Step | Action |
|------|--------|
| 1 | Wait for terminal to show "All nodes are active" |
| 2 | In RViz, click **"2D Pose Estimate"** button |
| 3 | Click on map where robot actually is, drag arrow for heading |
| 4 | Click **"2D Goal Pose"** button |
| 5 | Click where you want robot to go, drag arrow for final orientation |
| 6 | Watch robot navigate autonomously! |

**What you see in RViz:**

| Color | Meaning |
|-------|---------|
| Gray | Map (free space) |
| Black | Walls/obstacles from map |
| Red dots | Live LiDAR scan |
| Green line | Global planned path |
| Yellow line | Local path being followed |
| Purple/pink overlay | Costmap (inflation around obstacles) |

---

### Adjusting Navigation Speed

Edit the file `src/herm_bringup/config/nav2_params.yaml`:

**Controller velocities (main speed settings):**
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      min_vel_x: 0.22      # Minimum forward speed (m/s)
      max_vel_x: 0.75      # Maximum forward speed (m/s)
      max_vel_theta: 3.0   # Maximum rotation speed (rad/s)
      min_speed_xy: 0.22   # Minimum linear speed
      max_speed_xy: 0.75   # Maximum linear speed
      min_speed_theta: 0.6 # Minimum rotation speed
```

**Velocity smoother (acceleration limits):**
```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.75, 0.0, 3.0]    # [linear_x, linear_y, angular_z]
    min_velocity: [-0.75, 0.0, -3.0]  # For reverse
    max_accel: [3.0, 0.0, 6.0]        # Acceleration limits
    max_decel: [-3.0, 0.0, -6.0]      # Deceleration limits
```

**After editing, rebuild:**
```bash
cd ~/herm_ws
colcon build --packages-select herm_bringup --symlink-install
source install/setup.bash
```

**Speed recommendations:**

| Environment | max_vel_x | max_vel_theta |
|-------------|-----------|---------------|
| Tight indoor | 0.3 | 1.5 |
| Normal indoor | 0.5 | 2.0 |
| Open space | 0.75 | 3.0 |
| Fast (be careful!) | 1.0 | 4.0 |

---

### Quick Reference - All Commands

**SLAM (mapping):**
```bash
# Terminal 1: Launch SLAM
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup slam_test.launch.py

# Terminal 2: Save map (while SLAM is running)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_home
```

**Navigation:**
```bash
# Single terminal - launch everything
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py map:=/home/jithin/maps/my_home.yaml
```

**Useful debug commands (run in separate terminal):**
```bash
# Check if velocity commands are being sent
ros2 topic echo /cmd_vel

# Check robot's estimated position
ros2 topic echo /amcl_pose

# Check if path is planned
ros2 topic echo /plan

# List all active topics
ros2 topic list

# Check Nav2 node status
ros2 lifecycle list /bt_navigator
```

---

### Troubleshooting Navigation

| Problem | Solution |
|---------|----------|
| "Empty Tree" error | Behavior tree not found - make sure nav2_params.yaml has correct BT path |
| Robot doesn't move | Check `/cmd_vel` topic. If values show, increase `min_vel_x` |
| Motor sounds but no movement | Velocities too low - increase min speeds in nav2_params.yaml |
| "Fixed frame map does not exist" | Wait for map_server to load, or check map file path |
| TF errors at startup | Normal during initialization - wait a few seconds |
| Path planning fails | Goal might be in obstacle. Try different location |
| Robot spins in place | Set initial pose with "2D Pose Estimate" first |
| AMCL not localizing | Drive robot around a bit to help particle filter converge |

---

### Launch File Reference

| Launch File | Purpose |
|-------------|---------|
| `slam_test.launch.py` | SLAM mapping with Xbox teleop |
| `navigation.launch.py` | Autonomous navigation with Nav2 |
| `odom_test.launch.py` | Test/calibrate odometry |
| `l298n_simple.launch.py` | Simple motor test with Xbox |

---

## Troubleshooting

See [ERRORS.md](ERRORS.md) for a comprehensive list of common errors and fixes, including:
- ROS2 launch and build errors
- Xbox controller connection issues
- Serial/I2C device problems
- Camera and LiDAR troubleshooting
- Gazebo simulation fixes

---

## Notes

A few things worth knowing:

- The simulation runs on Gazebo Harmonic (the new Gazebo, not Gazebo Classic)
- On Jetson, the camera uses GStreamer because the Insta360 streams MJPEG and standard V4L2 drivers don't handle it well
- RPLidar A2M12 needs 256000 baud rate, not the default 115200
- BNO055 IMU uses I2C bus 7 on Jetson Orin Nano (may vary on other boards)

---

## License

Apache 2.0 — use it, modify it, build on it.

---

## Stay Updated

All updates, experiments, and progress will be shared in this repository. Star or watch the repo if you want to follow along.
