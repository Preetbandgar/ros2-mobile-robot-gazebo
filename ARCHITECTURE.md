# Package Architecture Documentation

## Overview

This document provides detailed information about the architecture, design decisions, and component interactions in the ROS2 Mobile Robot Gazebo project.

**Course Context:** This project was developed following the **ROS2 For Beginners (Level 2) - TF, URDF, RViz, Gazebo** course by **Edouard Renard (Udemy)**.

**Control Paradigm:** This is a **teleoperated system** where the robot is manually controlled via keyboard. The robot does not have autonomous navigation capabilities in its current implementation.

**Robot Description:** Differential-drive mobile robot with a simple 2-DOF arm (two revolute joints) and an RGB camera sensor.

**Technology Stack:**
- ROS2 Jazzy Jalisco
- Gazebo Harmonic  
- Ubuntu 24.04 LTS
- Python 3.12

## Package Overview

### 1. my_robot_description

**Purpose**: Contains all robot model definitions, visual/collision geometries, and kinematic descriptions.

**Key Components**:

#### URDF/Xacro Files

1. **my_robot.urdf.xacro** (Main Assembly)
   - Root file that includes all other xacro files
   - Defines the connection between mobile base and arm
   - Acts as the entry point for robot description

2. **mobile_base.xacro** (Mobile Platform)
   - Defines differential drive base
   - Properties:
     - Base: 0.6m × 0.4m × 0.2m box
     - Mass: 5.0 kg
     - Two motorized wheels (0.1m radius)
     - One caster wheel (0.05m radius)
   - Joints:
     - `base_right_wheel_joint` (continuous)
     - `base_left_wheel_joint` (continuous)
     - `base_caster_wheel_joint` (fixed)

3. **mobile_base_gazebo.xacro** (Mobile Base Plugins)
   - Differential drive plugin configuration
   - Wheel physics parameters
   - Odometry publishing settings

4. **arm.xacro** (Robotic Arm)
   - 2-DOF manipulator definition
   - Links:
     - `arm_base_link`: Base of the arm
     - `forearm_link`: Middle section
     - `hand_link`: End effector
   - Joints:
     - `arm_base_forearm_joint` (revolute)
     - `forearm_hand_joint` (revolute)

5. **arm_gazebo.xacro** (Arm Plugins)
   - Joint position controllers
   - PID parameters for each joint
   - Gazebo physics properties

6. **camera.xacro** (Camera Sensor)
   - RGB camera definition
   - Optical frame setup
   - Gazebo camera plugin configuration
   - Image publishing parameters

7. **common_properties.xacro** (Shared Macros)
   - Material definitions (colors)
   - Inertia calculation macros:
     - `box_inertia`
     - `cylinder_inertia`
     - `sphere_inertia`
   - Reduces code duplication

#### Launch Files

1. **display.launch.py**
   - Python-based launch file
   - Starts RViz2 with robot model
   - Useful for URDF verification without Gazebo

2. **display.launch.xml**
   - XML-based alternative to display.launch.py
   - Same functionality, different format

#### RViz Configuration

1. **urdf_config.rviz**
   - Pre-configured RViz settings
   - Display configurations:
     - RobotModel
     - TF frames
     - Camera view
     - Grid
     - Axes

### 2. my_robot_bringup

**Purpose**: Orchestrates the complete simulation environment, including launch files, world files, and configuration.

**Key Components**:

#### Launch Files

1. **my_robot_gazebo.launch.xml**
   - Master launch file for complete simulation
   - Components launched:
     - `robot_state_publisher`: Publishes robot TF tree
     - `joint_state_publisher_gui`: Manual joint control
     - `gz_sim`: Gazebo simulator with custom world
     - `create`: Spawns robot in Gazebo
     - `ros_gz_bridge`: Bridges ROS2 and Gazebo topics
     - `rviz2`: Visualization with pre-configured layout

#### Configuration Files

1. **gazebo_bridge.yaml**
   - Defines ROS2 ↔ Gazebo topic mappings
   - Bridge configurations:
     ```yaml
     Direction: GZ_TO_ROS (Gazebo → ROS2)
     - /clock
     - /joint_states
     - /tf
     - /camera/camera_info
     - /camera/image_raw
     
     Direction: ROS_TO_GZ (ROS2 → Gazebo)
     - /cmd_vel
     - /joint0/cmd_pos
     - /joint1/cmd_pos
     ```

#### World Files

1. **test_world.sdf**
   - Custom Gazebo world definition
   - Contains environment elements
   - Lighting configuration
   - Physics engine settings

## System Architecture

### Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                          ROS2 Layer                              │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐         ┌─────────────────┐                  │
│  │  Teleop      │────────▶│   /cmd_vel      │                  │
│  │  Keyboard    │         │  (Twist msgs)   │                  │
│  │  (Manual)    │         │                 │                  │
│  └──────────────┘         └─────────────────┘                  │
│        ↑                          │                             │
│        │                          ▼                             │
│   User Input              ┌──────────────┐                      │
│                           │  ROS-GZ      │                      │
│                           │  Bridge      │                      │
│                           └──────────────┘                      │
│                                  │                              │
├──────────────────────────────────┼───────────────────────────────┤
│                                  ▼                              │
│                       Gazebo Harmonic                           │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              Robot Model (URDF)                           │  │
│  │  ┌──────────┐    ┌─────────┐    ┌──────────┐            │  │
│  │  │ Mobile   │    │  Arm    │    │  Camera  │            │  │
│  │  │ Base     │────│ (2-DOF) │    │  Sensor  │            │  │
│  │  │(Manual)  │    │         │    │          │            │  │
│  │  └──────────┘    └─────────┘    └──────────┘            │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                │                                │
│                                ▼                                │
│                  ┌──────────────────────────┐                   │
│                  │  Physics Engine          │                   │
│                  │  (Collision, Dynamics)   │                   │
│                  └──────────────────────────┘                   │
│                                │                                │
├────────────────────────────────┼─────────────────────────────────┤
│                                ▼                                │
│                         ┌──────────────┐                        │
│                         │  ROS-GZ      │                        │
│                         │  Bridge      │                        │
│                         └──────────────┘                        │
│                                │                                │
├────────────────────────────────┼─────────────────────────────────┤
│                         ROS2 Layer                              │
│                                │                                │
│                                ▼                                │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │/joint_states│    │     /tf      │    │  /camera/    │      │
│  │             │    │              │    │  image_raw   │      │
│  └─────────────┘    └──────────────┘    └──────────────┘      │
│         │                   │                    │             │
│         ▼                   ▼                    ▼             │
│  ┌───────────────────────────────────────────────────────┐    │
│  │          robot_state_publisher                        │    │
│  │          (Publishes TF tree)                          │    │
│  └───────────────────────────────────────────────────────┘    │
│                                │                               │
│                                ▼                               │
│                          ┌──────────┐                          │
│                          │  RViz2   │                          │
│                          │ (Visual) │                          │
│                          └──────────┘                          │
└──────────────────────────────────────────────────────────────────┘

Note: User manually controls robot via teleop_twist_keyboard
```

### Node Communication

#### Publishers

| Node | Topic | Type | Purpose |
|------|-------|------|---------|
| `robot_state_publisher` | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot transforms |
| `gz_bridge` | `/joint_states` | `sensor_msgs/msg/JointState` | Joint positions |
| `gz_bridge` | `/camera/image_raw` | `sensor_msgs/msg/Image` | Camera feed |
| `gz_bridge` | `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |

#### Subscribers

| Node | Topic | Type | Purpose |
|------|-------|------|---------|
| `gz_bridge` | `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `gz_bridge` | `/joint0/cmd_pos` | `std_msgs/msg/Float64` | Arm joint 0 position |
| `gz_bridge` | `/joint1/cmd_pos` | `std_msgs/msg/Float64` | Arm joint 1 position |
| `robot_state_publisher` | `/joint_states` | `sensor_msgs/msg/JointState` | For TF calculation |

## TF Tree Structure

```
base_footprint (Fixed on ground)
    │
    └─── base_link (Main robot body)
            │
            ├─── right_wheel_link (Continuous joint)
            │
            ├─── left_wheel_link (Continuous joint)
            │
            ├─── caster_wheel_link (Fixed joint)
            │
            ├─── camera_link (Fixed joint)
            │       │
            │       └─── camera_link_optical (Fixed, rotated frame)
            │
            └─── arm_base_link (Fixed joint)
                    │
                    └─── forearm_link (Revolute joint)
                            │
                            └─── hand_link (Revolute joint)
```

### Frame Transformations

- **base_footprint → base_link**: Offset by wheel radius (0.1m in Z)
- **base_link → wheel_links**: Positioned at wheel mounting points
- **base_link → arm_base_link**: Positioned at front-quarter of base
- **arm_base_link → forearm_link**: Revolute, angle controlled by joint0
- **forearm_link → hand_link**: Revolute, angle controlled by joint1
- **base_link → camera_link**: Fixed position and orientation

## Gazebo Plugins

### 1. Differential Drive Plugin

Located in: `mobile_base_gazebo.xacro`

**Configuration**:
- Left wheel joint: `base_left_wheel_joint`
- Right wheel joint: `base_right_wheel_joint`
- Wheel separation: ~0.45m
- Wheel diameter: 0.2m
- Command topic: `/cmd_vel`
- Odometry topic: `/odom`
- TF publishing: Enabled

### 2. Joint Position Controllers

Located in: `arm_gazebo.xacro`

**Configuration for each joint**:
- PID gains (P, I, D)
- Position command topics
- Effort limits
- Velocity limits

### 3. Camera Plugin

Located in: `camera.xacro`

**Configuration**:
- Image size: Configurable (e.g., 640×480)
- Update rate: 30 Hz
- Field of view: Configurable
- Topics:
  - `/camera/image_raw`
  - `/camera/camera_info`

## Build System

### CMakeLists.txt Structure

Both packages use `ament_cmake` build type:

```cmake
cmake_minimum_required(VERSION 3.8)
project(package_name)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
```

### Package Dependencies

**my_robot_description**:
- `ament_cmake` (build)
- No runtime dependencies (pure description package)

**my_robot_bringup**:
- `ament_cmake` (build)
- `robot_state_publisher` (exec)
- `ros_gz_sim` (exec)
- `ros_gz_bridge` (exec)
- `rviz2` (exec)

## Design Decisions

### Why Xacro?

1. **Modularity**: Separate files for each component
2. **Reusability**: Macros for repeated patterns
3. **Maintainability**: Easy to modify individual components
4. **Parameterization**: Properties for easy configuration

### Why XML Launch Files?

While Python launch files offer more flexibility, XML launch files are:
- Easier to read for simple configurations
- Declarative and intuitive
- Sufficient for this project's needs
- Can be easily converted to Python if needed

### Bridge Configuration

The bridge is configured via YAML instead of launch arguments for:
- Better organization
- Easy modification without recompiling
- Clear topic mapping visualization
- Easier to add new topics

## Performance Considerations

### Simulation Performance

- **Physics update rate**: Default 1000 Hz
- **RTF (Real-Time Factor)**: Depends on hardware
- **Optimization tips**:
  - Reduce sensor update rates if needed
  - Simplify collision geometries
  - Use appropriate solver parameters

### Memory Usage

- Gazebo: ~500MB-1GB
- ROS2 nodes: ~200MB
- Total: ~1-1.5GB typical

## Extension Points

### Adding New Sensors

1. Create new xacro file (e.g., `lidar.xacro`)
2. Include in `my_robot.urdf.xacro`
3. Add Gazebo plugin configuration
4. Update `gazebo_bridge.yaml` for topic bridging
5. Rebuild and test

### Adding New Joints

1. Define joint in appropriate xacro file
2. Add Gazebo plugin for control
3. Add bridge configuration
4. Update controller config if needed

### Custom Worlds

1. Create new `.sdf` file in `worlds/`
2. Update launch file to use new world
3. Test spawning and interactions

## Troubleshooting Guide

### Common Issues

1. **TF Tree Broken**
   - Check `robot_state_publisher` is running
   - Verify joint_states are published
   - Ensure all joints are defined in URDF

2. **Bridge Not Working**
   - Check topic names match exactly
   - Verify message types are correct
   - Ensure gz_bridge node is running

3. **Robot Falls Through Ground**
   - Check collision geometries exist
   - Verify inertial properties are set
   - Ensure world has ground plane

4. **Joints Not Moving**
   - Check PID gains are reasonable
   - Verify effort limits are not too low
   - Ensure commands are being received

## Testing Procedures

### 1. URDF Validation
```bash
check_urdf robot.urdf
urdf_to_graphiz robot.urdf
```

### 2. TF Tree Verification
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_link
```

### 3. Topic Communication
```bash
ros2 topic list
ros2 topic hz /joint_states
ros2 topic echo /camera/image_raw --once
```

### 4. Node Health
```bash
ros2 node list
ros2 node info /robot_state_publisher
```

## Future Improvements

**Current State:** The system is teleoperated with manual keyboard control. The following improvements would add autonomous capabilities:

1. **Navigation Stack Integration**
   - Add SLAM capabilities
   - Implement path planning
   - Add obstacle avoidance
   - Transform from teleoperated to autonomous

2. **Advanced Sensors**
   - LiDAR/Laser scanner
   - IMU sensor
   - Depth camera

3. **Control Improvements**
   - MoveIt integration for arm
   - Advanced differential drive control
   - Waypoint navigation

4. **Simulation Enhancements**
   - Physics parameter tuning
   - More complex worlds
   - Multi-robot support

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [ros_gz Repository](https://github.com/gazebosim/ros_gz)
- [URDF Documentation](http://wiki.ros.org/urdf)
- [Xacro Documentation](http://wiki.ros.org/xacro)

---

For more information, refer to the main [README.md](README.md) or [QUICKSTART.md](QUICKSTART.md).
