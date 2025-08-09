# ROS2 Navigation Stack
ROS 2-based navigation system with mapping, path planning, and path following nodes.
A user can set a goal position, and the robot will autonomously navigate to it while avoiding obstacles along the way.


## ✨ Features
- **Mapping** – Generates a map of the environment using onboard sensors.
- **Path Planning** – Calculates an optimal path from the robot’s current position to the goal **using the A\* algorithm** on a **costmap**.
- **Path Following** – Executes the planned path, dynamically avoiding obstacles, with **emergency stop** if a new obstacle appears directly in front of the robot.
- Supports **goal input via RViz** or custom interfaces.

## 🖼 Example Output
The path planning module uses **A\*** on a **costmap** to compute an optimal route to the goal.  
- **Black line** – Path computed by the A\* algorithm.  
- **Red line** – Interpolated path for smoother following.
- 
<img width="866" height="712" alt="irl_path" src="https://github.com/user-attachments/assets/d7c6757e-fa3b-47fa-b486-0e01405aaa26" />

The following plot compares the **desired** and **measured** robot orientation during path following, as well as the tracking error.

<img width="989" height="490" alt="plot" src="https://github.com/user-attachments/assets/794058d8-057f-4797-9e23-67d4b96a4923" />

## 🎥 Demo

![video](https://github.com/user-attachments/assets/15e12ba5-9a2d-4308-a00e-01fca92da954)


## 📦 Requirements
- ROS 2
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`
- A robot setup with odometry and sensor data (e.g., LiDAR)

## ▶️ Usage

### Start mapping
```bash
ros2 launch mapping mapping.launch.py  
```

### Start path_planning
```bash
ros2 launch path_planning path_planning.launch.py
```

### Start path_following
```bash
ros2 launch path_following path_following.launch.py 
```


