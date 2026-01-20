# Remote Access Guide

Control your HERM robot from another PC over the network.

---

## Quick Start

**From your laptop/desktop:**
```bash
ssh jithin@<ROBOT_IP>
```

**Then run robot commands as normal:**
```bash
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py
```

---

## Step 1: Find Robot's IP Address

**On the robot (Jetson):**
```bash
hostname -I
```

Or:
```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
```

Example output: `192.168.1.105` - this is your ROBOT_IP.

**Tip:** Set a static IP on your router for the Jetson so it doesn't change.

---

## Step 2: Enable SSH on Robot

SSH should be enabled by default on Ubuntu. If not:

```bash
# On the robot
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

**Check SSH status:**
```bash
sudo systemctl status ssh
```

---

## Step 3: Connect from Your PC

**Basic connection:**
```bash
ssh jithin@192.168.1.105
```

**With password-less login (recommended):**

On your PC, generate SSH key (skip if you already have one):
```bash
ssh-keygen -t ed25519
```

Copy key to robot:
```bash
ssh-copy-id jithin@192.168.1.105
```

Now you can connect without password:
```bash
ssh jithin@192.168.1.105
```

---

## Step 4: Running ROS2 Remotely

### Option A: Run Everything on Robot (Simple)

SSH into robot and run commands there:
```bash
ssh jithin@192.168.1.105
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py
```

RViz runs on the robot's display. You won't see it on your PC.

---

### Option B: View RViz on Your PC (Recommended)

Run robot nodes on Jetson, run RViz on your PC.

**Prerequisites on your PC:**
```bash
# Install ROS2 Humble on your PC
# Install Nav2 for rviz plugins
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

**Step 1: Set ROS_DOMAIN_ID (same on both machines)**

On robot:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

On your PC:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**Step 2: Launch robot nodes (SSH into robot)**
```bash
ssh jithin@192.168.1.105
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py use_rviz:=false
```

Wait, we need to add use_rviz parameter. For now, just let RViz fail on robot or kill it:
```bash
# Launch navigation (RViz will fail if no display, that's OK)
ros2 launch herm_bringup navigation.launch.py
```

**Step 3: Run RViz on your PC**
```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

Add displays manually:
- Map: `/map`
- LaserScan: `/scan`
- RobotModel: `/robot_description`
- Path: `/plan`

Or copy the rviz config from robot and use it.

---

### Option C: X11 Forwarding (See Robot's GUI on Your PC)

**Connect with X11 forwarding:**
```bash
ssh -X jithin@192.168.1.105
```

Or for better performance:
```bash
ssh -Y jithin@192.168.1.105
```

**Then run RViz:**
```bash
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py
```

RViz window appears on your PC but runs on robot.

**Note:** X11 forwarding can be slow over WiFi. Option B is faster.

---

## Useful SSH Commands

**Run command without staying connected:**
```bash
ssh jithin@192.168.1.105 "source ~/herm_ws/install/setup.bash && ros2 topic list"
```

**Keep process running after disconnect (using screen):**
```bash
# On robot, install screen
sudo apt install screen

# Start a screen session
ssh jithin@192.168.1.105
screen -S robot

# Run your commands
source ~/herm_ws/install/setup.bash
ros2 launch herm_bringup navigation.launch.py

# Detach: Press Ctrl+A, then D
# Disconnect SSH - robot keeps running!

# Reconnect later
ssh jithin@192.168.1.105
screen -r robot
```

**Using tmux (alternative to screen):**
```bash
sudo apt install tmux

# Start tmux session
tmux new -s robot

# Run commands...

# Detach: Press Ctrl+B, then D

# Reattach later
tmux attach -t robot
```

---

## Multi-Machine ROS2 Setup

Run some nodes on robot, some on PC.

### Network Setup

Both machines must:
1. Be on same network
2. Be able to ping each other
3. Have same ROS_DOMAIN_ID

**Test connectivity:**
```bash
# From PC
ping 192.168.1.105

# From robot
ping <YOUR_PC_IP>
```

### Environment Setup

**On robot (~/.bashrc):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/herm_ws/install/setup.bash
```

**On your PC (~/.bashrc):**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
```

### Example: Robot + PC Split

**Terminal 1 (SSH to robot) - Core robot:**
```bash
ssh jithin@192.168.1.105
source ~/herm_ws/install/setup.bash

# Launch just the robot hardware
ros2 launch herm_bringup slam_test.launch.py
```

**Terminal 2 (Your PC) - Visualization:**
```bash
source /opt/ros/humble/setup.bash
rviz2
```

Topics from robot automatically appear on PC!

---

## Checking Connection

**On your PC, verify you see robot's topics:**
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
```

You should see:
```
/scan
/odom
/cmd_vel
/map
/tf
...
```

**Check nodes:**
```bash
ros2 node list
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Can't SSH | Check IP, ensure SSH is running: `sudo systemctl status ssh` |
| Connection refused | Firewall might be blocking. Try: `sudo ufw allow ssh` |
| ROS2 topics not visible | Check ROS_DOMAIN_ID matches on both machines |
| X11 forwarding slow | Use Option B (run RViz locally) instead |
| "Display not set" error | Use `ssh -X` or set `export DISPLAY=:0` on robot |
| Network timeout | Ensure both on same subnet, check firewall |

---

## SSH Config (Optional)

Create `~/.ssh/config` on your PC for easy connection:

```
Host herm
    HostName 192.168.1.105
    User jithin
    ForwardX11 yes
```

Now just type:
```bash
ssh herm
```

---

## Quick Reference

```bash
# Connect to robot
ssh jithin@<ROBOT_IP>

# Connect with GUI forwarding
ssh -X jithin@<ROBOT_IP>

# Copy file to robot
scp myfile.txt jithin@<ROBOT_IP>:~/

# Copy file from robot
scp jithin@<ROBOT_IP>:~/data.bag ./

# Copy entire folder
scp -r myfolder/ jithin@<ROBOT_IP>:~/

# Run RViz on PC, see robot data
export ROS_DOMAIN_ID=42
rviz2
```

---

## Recommended Workflow

1. **SSH Terminal 1:** Launch navigation on robot
2. **SSH Terminal 2:** Monitor logs / run debug commands
3. **PC Terminal:** Run RViz locally for smooth visualization

This gives you the best performance - heavy computation on Jetson, smooth graphics on your PC.


  What's included:                                                                    
  - Finding robot's IP                                                                
  - SSH setup and password-less login                                                 
  - Running ROS2 remotely                                                             
  - X11 forwarding for GUI                                                            
  - Multi-machine ROS2 setup (run RViz on PC, nodes on robot)                         
  - Using screen/tmux to keep processes running                                       
  - SSH config shortcuts                                                              
  - Troubleshooting                                                                   
                                                                                      
  Also added use_rviz parameter to launch files:                                      
                                                                                      
  # Run headless on robot (no RViz)                                                   
  ros2 launch herm_bringup navigation.launch.py use_rviz:=false                       
                                                                                      
  # Run RViz on your PC                                                               
  export ROS_DOMAIN_ID=42                                                             
  rviz2                                                                               
                                                                                      
  View the guide:                                                                     
  cat ~/herm_ws/REMOTE.md                                                             
                                                                                      
  Quick start from your other PC:                                                     
  # 1. SSH to robot                                                                   
  ssh jithin@<ROBOT_IP>                                                               
                                                                                      
  # 2. Run navigation headless                                                        
  source ~/herm_ws/install/setup.bash                                                 
  ros2 launch herm_bringup navigation.launch.py use_rviz:=false                       
                                                                                      
  # 3. On your PC - run RViz locally (smoother)                                       
  export ROS_DOMAIN_ID=42                                                             
  rviz2                                                                               
              