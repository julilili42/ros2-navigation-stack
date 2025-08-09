# ROS2 Navigation Stack
ROS 2-based navigation system with mapping, path planning, and path following nodes.
A user can set a goal position, and the robot will autonomously navigate to it while avoiding obstacles along the way.


## ✨ Features
- **Mapping** – Generates a map of the environment using onboard sensors.
- **Path Planning** – Calculates an optimal path from the robot’s current position to the goal.
- **Path Following** – Executes the planned path, dynamically avoiding obstacles.
- Supports **goal input via RViz** or custom interfaces.


## 📦 Requirements
- ROS 2
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`
- A robot setup with odometry and sensor data (e.g., LiDAR)


## Build
cd ~/ros2_ws
colcon build
source install/setup.bash


## 🎥 Demo

![video](https://github.com/user-attachments/assets/15e12ba5-9a2d-4308-a00e-01fca92da954)
