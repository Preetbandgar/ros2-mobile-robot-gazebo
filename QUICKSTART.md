# Quick Start Guide - ROS2 Mobile Robot Gazebo

> **Course Project**: ROS2 For Beginners (Level 2) - TF, URDF, RViz, Gazebo by Edouard Renard (Udemy)

> **Note:** This robot is **teleoperated** (manually controlled). You need to run `teleop_twist_keyboard` to move it.

> **Robot:** Differential-drive mobile base with a simple 2-DOF arm and camera sensor.

## ⚡ Quick Installation (Ubuntu 24.04 + ROS2 Jazzy)

```bash
# 1. Install ROS2 Jazzy (if not already installed)
sudo apt update && sudo apt install ros-jazzy-desktop

# 2. Install Gazebo Harmonic and dependencies
sudo apt install -y \
  gz-harmonic \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-tools \
  ros-jazzy-teleop-twist-keyboard

# 3. Setup workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Extract or clone your project here

# 4. Build
cd ~/ros2_ws
colcon build
source install/setup.bash

# 5. Launch simulation
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

## 🚀 Most Common Commands

### Launch Simulation
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

### Control Robot (Keyboard) - **REQUIRED TO MOVE ROBOT**
```bash
# New terminal - robot won't move without this!
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Keys:** `i`=forward, `k`=stop, `j`=left, `l`=right, `,`=backward

### Control Arm Joints
```bash
# Joint 0 (base to forearm)
ros2 topic pub /joint0/cmd_pos std_msgs/msg/Float64 "data: 1.57"

# Joint 1 (forearm to hand)
ros2 topic pub /joint1/cmd_pos std_msgs/msg/Float64 "data: 0.785"
```

### View Topics
```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic hz /camera/image_raw
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Debug
```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic list

# View ROS graph
rqt_graph

# Monitor TF
ros2 run tf2_ros tf2_monitor
```

## 🔧 Quick Fixes

### Gazebo Won't Start
```bash
killall -9 gz
rm -rf ~/.gz/sim
```

### Robot Not Spawning
```bash
ros2 run ros_gz_sim create -topic robot_description
```

### Rebuild Project
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

## 📋 Package Structure
```
my_robot_description/   # URDF files, robot model
my_robot_bringup/       # Launch files, worlds, configs
```

## 🎯 Key Files
- **URDF**: `my_robot_description/urdf/my_robot.urdf.xacro`
- **Launch**: `my_robot_bringup/launch/my_robot_gazebo.launch.xml`
- **Bridge Config**: `my_robot_bringup/config/gazebo_bridge.yaml`
- **World**: `my_robot_bringup/worlds/test_world.sdf`
- **RViz Config**: `my_robot_description/rviz/urdf_config.rviz`

## 💡 Useful Tips

1. **Always source workspace**: `source ~/ros2_ws/install/setup.bash`
2. **Add to .bashrc**: `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`
3. **Use symlink install**: `colcon build --symlink-install`
4. **Check ROS2 is working**: `ros2 doctor`
5. **View available worlds**: `gz sim -l`

## 🔌 Important Topics

| Topic | Purpose |
|-------|---------|
| `/cmd_vel` | Move robot |
| `/joint_states` | Current joint positions |
| `/camera/image_raw` | Camera feed |
| `/joint0/cmd_pos` | Control arm joint 0 |
| `/joint1/cmd_pos` | Control arm joint 1 |
| `/tf` | Transform tree |

## 🎮 Keyboard Controls (teleop_twist_keyboard)

```
        i    
    j   k   l
        ,    

i: Forward
k: Stop
j: Turn left
l: Turn right
,: Backward
u/o/m/.: Diagonal movements
```

## 📊 Check System Status

```bash
# ROS2 environment
ros2 doctor

# Active nodes
ros2 node list

# Topics with publishers/subscribers
ros2 topic list -v

# Available services
ros2 service list

# TF tree status
ros2 run tf2_ros tf2_monitor
```

## 🛑 Emergency Stop

```bash
# Stop all ROS nodes
killall -9 ros2

# Stop Gazebo
killall -9 gz

# Clean restart
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

---

For detailed information, see the main [README.md](README.md)
