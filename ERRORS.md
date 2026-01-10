# HERM - Common Errors and Fixes

A troubleshooting guide for common issues encountered while working with the HERM robot.

---

## ROS2 / Launch Errors

### Executable not found on libexec directory

**Error:**
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): executable 'camera_node.py' not found on the libexec directory '/home/user/herm_ws/install/herm_bringup/lib/herm_bringup'  ros2 launch herm_bringup sensors_test.launch.py

```

**Cause:** Python scripts are not marked as executable.

**Fix:**
```bash
chmod +x ~/herm_ws/src/herm_bringup/scripts/*.py
colcon build --packages-select herm_bringup --symlink-install
source ~/herm_ws/install/setup.bash
```

---

### Package not found

**Error:**
```
Package 'herm_bringup' not found
```

**Cause:** Workspace not sourced or package not built.

**Fix:**
```bash
cd ~/herm_ws
colcon build
source install/setup.bash
```

---

### Topic not being published

**Error:** `ros2 topic echo /topic_name` shows nothing or "WARNING: topic does not appear to be published yet"

**Cause:** Node not running or wrong topic name.

**Fix:**
```bash
# List all active topics
ros2 topic list

# Check if node is running
ros2 node list

# Verify topic info
ros2 topic info /topic_name
```

---

## Xbox Controller Errors

### Controller LED keeps blinking (not connected)

**Cause:** Controller paired but not trusted, or ERTM not disabled.

**Fix:**
```bash
# Disable ERTM
sudo bash -c 'echo 1 > /sys/module/bluetooth/parameters/disable_ertm'

# Trust the controller
bluetoothctl trust <MAC_ADDRESS>
bluetoothctl connect <MAC_ADDRESS>
```

---

### Controller connects then immediately disconnects

**Cause:** Outdated controller firmware or xpadneo driver issues.

**Fix:**
1. Update Xbox controller firmware using Windows PC or Xbox console
2. Update xpadneo driver:
```bash
cd /tmp
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo
sudo ./uninstall.sh
sudo ./install.sh
sudo reboot
```

---

### No joystick input in ROS2

**Error:** `ros2 topic echo /joy` shows no data or all zeros.

**Cause:** Joy node not running or wrong device.

**Fix:**
```bash
# Check if joystick exists
ls /dev/input/js*

# Test joystick directly
jstest /dev/input/js0

# Run joy node with correct device
ros2 run joy joy_node --ros-args -p device_id:=0
```

---

### Bluetooth scan fails

**Error:**
```
Failed to start discovery: org.bluez.Error.NotReady
```

**Cause:** Bluetooth adapter not ready.

**Fix:**
```bash
# Restart Bluetooth service
sudo systemctl restart bluetooth

# Power on adapter in bluetoothctl
bluetoothctl
power on
scan on
```

---

## Serial / Hardware Errors

### Permission denied on /dev/ttyUSB0

**Error:**
```
serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyUSB0: Permission denied
```

**Fix:**
```bash
# Temporary fix
sudo chmod 666 /dev/ttyUSB0

# Permanent fix (add user to dialout group)
sudo usermod -aG dialout $USER
# Then logout and login again
```

---

### Device not found /dev/ttyUSB0

**Cause:** USB device not connected or different port assigned.

**Fix:**
```bash
# List USB devices
ls /dev/ttyUSB*

# Check dmesg for device assignment
dmesg | grep ttyUSB
```

---

### I2C device not found (BNO055)

**Error:**
```
OSError: [Errno 121] Remote I/O error
```

**Cause:** Wrong I2C bus or device not connected.

**Fix:**
```bash
# Scan for I2C devices
sudo i2cdetect -y 7   # Try different bus numbers (0, 1, 7, 8)

# BNO055 should appear at address 0x28 or 0x29
```

---

## Gazebo Simulation Errors

### Gazebo won't start / black screen

**Cause:** GPU driver issues or missing dependencies.

**Fix:**
```bash
# Install dependencies
sudo apt install ros-humble-ros-gz

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch herm_simulation simulation.launch.py
```

---

### Robot not spawning in Gazebo

**Cause:** URDF/SDF errors or bridge not running.

**Fix:**
```bash
# Check for URDF errors
check_urdf ~/herm_ws/src/herm_description/urdf/herm.urdf.xacro

# Verify Gazebo bridge is running
ros2 topic list | grep gz
```

---

## Camera Errors

### GStreamer pipeline error

**Error:**
```
GStreamer warning: Cannot query video position
```

**Cause:** Wrong video device or format not supported.

**Fix:**
```bash
# List video devices
ls /dev/video*

# Check camera capabilities
v4l2-ctl --list-formats -d /dev/video0

# Try different device
ros2 run herm_bringup camera_node.py --ros-args -p device:=/dev/video2
```

---

### Camera image is black/corrupted

**Cause:** Wrong resolution or format settings.

**Fix:**
```bash
# Test camera with cheese or similar app
cheese

# Check if camera works with v4l2
ffplay /dev/video0
```

---

## LiDAR Errors

### RPLidar not spinning

**Cause:** Wrong baud rate or insufficient power.

**Fix:**
```bash
# RPLidar A2M12 requires 256000 baud rate (not default 115200)
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000
```

---

### Scan data has large gaps

**Cause:** Motor speed issue or obstruction.

**Fix:**
- Check nothing is blocking the LiDAR view
- Ensure LiDAR is getting enough power (use powered USB hub if needed)
- Clean the sensor lens

---

## Build Errors

### CMake cannot find package

**Error:**
```
Could not find a package configuration file provided by "package_name"
```

**Fix:**
```bash
# Install missing ROS2 package
sudo apt install ros-humble-<package-name>

# Source ROS2
source /opt/ros/humble/setup.bash
```

---

### Python module not found

**Error:**
```
ModuleNotFoundError: No module named 'serial'
```

**Fix:**
```bash
pip3 install pyserial
# or
pip3 install smbus2  # for I2C
```

---

## Network / Multi-Machine Errors

### Nodes on different machines can't communicate

**Cause:** ROS_DOMAIN_ID mismatch or firewall blocking.

**Fix:**
```bash
# Set same domain ID on all machines
export ROS_DOMAIN_ID=42

# Or disable firewall (not recommended for production)
sudo ufw disable
```

---

## Quick Diagnostic Commands

```bash
# Check ROS2 environment
printenv | grep ROS

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /topic_name

# View node info
ros2 node info /node_name

# Check TF tree
ros2 run tf2_tools view_frames

# System diagnostics
ros2 run rqt_graph rqt_graph
```

---

## Still Stuck?

1. Check the ROS2 logs: `~/.ros/log/`
2. Run with debug output: `ros2 launch ... --debug`
3. Open an issue on the repository with:
   - Error message
   - Steps to reproduce
   - System info (`uname -a`, ROS2 version)
