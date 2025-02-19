# Custom robot with Nav2

A ROS 2 based robot featuring custom navigation nodes and optimized parameters for autonomous operation done using Zenoh middleware, can also be simulated using cycloneDDS, PLEASE NOTE THAT fastDDS makes the gazebo environment crash


## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Ignition Fortress
- RViz2
- Python 3.10+
- C++ 14 or newer

### Required Packages
```bash
# Install core dependencies
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
```
# Install Gazebo Ignition-Fortress
sudo apt-get install ignition-fortress


## System Components

### 1. Custom Navigation Node
The specialized navigation implementation includes:
- Enhanced path planning algorithms
- Custom cost functions
- Optimized obstacle avoidance
- Real-time path optimization

### 2. Simulation Environment
Fully integrated Gazebo simulation with:
- Warehouse environment
- Simulated sensors
- Performance monitoring
- Debug visualization

### 3. Localization System
Map-based positioning system using:
- AMCL localization
- Custom map integration
- Real-time position tracking

### 3. Navigation System using Nav2
Map-based navigation system using:
- AMCL localization
- Map integration
- Real-time dynamic obstacle avoidance

## Running the Complete System

### 1. Start the Zenoh Daemon (if using Zenoh)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### 2. Launch Core System
```bash
# Terminal 1: Launch simulation and robot
ros2 launch accl_task launch_sim.launch.py

# Terminal 2: Launch localization
ros2 launch accl_task localization_launch.py map:=./src/accl_task/maps/warehouse_save.yaml use_sim_time:=true

# Terminal 3: Launch custom navigation
ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true

# Terminal 4: Run custom navigation node
ros2 run accl_task navigation_node
```
## Launch Files and Navigation Node Explained

### launch_sim.launch.py
Launches simulation components:
- Gazebo environment
- Robot spawning
- Sensor simulation
- Basic parameters

### localization_launch.py
Initiates localization:
- Map loading
- AMCL setup
- Transform configuration

### navigation_launch.py
Starts navigation stack:
- Custom navigation node
- Parameter loading
- Path planning
- Obstacle avoidance

### navigation_node.cpp
- Interfaces with running AMCL node and sets initial pose
- Interfaces with Running Nav2 nodes and gives coordinates for the robot to traverse through
- Adds visual markers after reaching a goal

## Configuration Files

### nav_params.yaml
Fine tuned to match environmental constraints and robot capabilities

## Development Tools

### Building and Testing
```bash
# Build specific packages
colcon build --packages-select accl_task or colcon build --symlink-install

## Common Issues and Solutions

### Navigation Issues
1. **Map Not Loading**
   - Verify map path
   - Check file permissions
   - Confirm YAML format

2. **Navigation Failures**
   - Check transform tree
   - Verify localization
   - Review costmap parameters

3. **Simulation Issues**
   - Confirm use_sim_time settings
   - Check Gazebo launch
   - Verify sensor data
