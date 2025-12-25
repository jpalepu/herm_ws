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
| Motors | Hiwonder 4-channel encoder motor driver |
| LiDAR | RPLidar A2M12 |
| Camera | Insta360 Link 2 (360° webcam) |
| IMU | MPU3050 |

### Physical Specs

| Spec | Value |
|------|-------|
| Chassis | 40 x 30 x 8 cm |
| Wheel Radius | 6.5 cm |
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
| LiDAR integration | Done |
| Camera integration | Done |
| Gazebo simulation | Done |
| Motor control | In progress |
| IMU integration | In progress |
| Teleop (keyboard + Xbox) | Planned |
| SLAM | Planned |
| Nav2 integration | Planned |
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

## Notes

A few things worth knowing:

- The simulation runs on Gazebo Harmonic (the new Gazebo, not Gazebo Classic)
- On Jetson, the camera uses GStreamer because the Insta360 streams MJPEG and standard V4L2 drivers don't handle it well
- RPLidar A2M12 needs 256000 baud rate, not the default 115200

---

## License

Apache 2.0 — use it, modify it, build on it.

---

## Stay Updated

All updates, experiments, and progress will be shared in this repository. Star or watch the repo if you want to follow along.
