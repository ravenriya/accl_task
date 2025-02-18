# ROS 2 Navigation Node with Checkpoint Visualization

This ROS 2 package implements a navigation node that allows for goal-point navigation with visual checkpoint markers in RViz. The node enables users to input target coordinates, navigate to them, and visualizes both reached goals and predefined checkpoints.

## Features

- Interactive goal-point navigation through command-line input
- Visual markers for reached goals (green spheres)
- Special checkpoint markers with labels (blue cylinders)
- Real-time distance feedback during navigation
- Predefined checkpoint positions with custom names
- Automatic initial pose setting

## Prerequisites

- ROS 2 Humble or newer
- Nav2 stack installed
- C++ 14 or newer
- RViz2

## Dependencies

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>geometry_msgs</depend>
<depend>nav2_msgs</depend>
<depend>visualization_msgs</depend>
```

## Installation

1. Create a new ROS 2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone <your-repository-url>
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build --symlink-install
```

5. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Launch your robot's navigation stack (example with Turtlebot3):
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

2. In a new terminal, source your workspace and run the navigation node:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run your_package_name navigation_node
```

3. Open RViz2 (if not already running):
```bash
rviz2
```

4. In RViz2:
   - Add a Marker display type
   - Set the fixed frame to "map"
   - Set the Marker topic to "/visualization_marker"

5. In the terminal running the navigation node:
   - Enter X coordinate when prompted
   - Enter Y coordinate when prompted
   - The robot will navigate to the specified position

## Customizing Checkpoints

To modify the checkpoint positions and names, edit the `checkpoints` vector in the `NavigationNode` class:

```cpp
std::vector<Checkpoint> checkpoints = {
    {1.0, 1.0, "Checkpoint 1"},
    {2.0, 2.0, "Checkpoint 2"},
    {0.0, 0.0, "Home"}
};
```

## Marker Visualization

- **Goal Markers**: Green spheres that appear when any goal is reached
- **Checkpoint Markers**: Blue cylinders with text labels that appear when predefined checkpoints are reached
- Markers remain visible for 5 seconds by default (can be modified in the code)

## Node Details

- **Node Name**: navigation_node
- **Published Topics**:
  - `/initialpose` (geometry_msgs/msg/PoseWithCovarianceStamped)
  - `/visualization_marker` (visualization_msgs/msg/Marker)
- **Action Client**:
  - `navigate_to_pose` (nav2_msgs/action/NavigateToPose)

## Configuration

You can modify the following parameters in the code:

- Marker display duration (default: 5 seconds)
- Marker sizes and colors
- Checkpoint positions and names
- Initial pose settings
- Checkpoint detection threshold (default: 0.1 meters)

## Building and Testing

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select your_package_name

# Run tests (if implemented)
colcon test --packages-select your_package_name
```

## Common Issues and Solutions

1. **Navigation Server Not Found**
   - Ensure your navigation stack is running
   - Check that all ROS 2 dependencies are installed
   - Verify that your workspace is properly sourced

2. **Markers Not Visible in RViz**
   - Confirm that the Marker display type is added in RViz
   - Verify the correct topic is selected ("/visualization_marker")
   - Check that the fixed frame is set to "map"

3. **Initial Pose Issues**
   - Ensure the map is properly loaded
   - Verify that the initial pose coordinates match your map

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 Navigation Stack Documentation
- ROS 2 Community
- Nav2 Project Team
