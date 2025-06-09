# 🧭 NAV2 - ROS 2 Navigation Stack Overview

Welcome to your NAV2 setup guide and documentation. This README provides a structured overview of how to install, run, and interact with the Navigation2 stack on ROS 2 Humble with TurtleBot3, along with useful tools and examples.

---

## 🔧 Environment

* **ROS 2 Version:** Humble
* **OS:** Ubuntu 22.04

---

## 📦 Installation

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
sudo apt install python3-colcon-common-extensions
```

---

## 💡 Useful Terminal Shortcuts (Terminator)

* **Split Vertically:** `Ctrl + Shift + O`
* **Split Horizontally:** `Ctrl + Shift + E`

---

## 🗺️ Launch Sample Worlds

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

## ⌨️ Teleoperation

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## 🛠️ SLAM with Cartographer

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

## 💾 Save a Map

```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

* **Legend:**

  * White: Known areas
  * Black: Obstacles
  * Grey: Unknown

---

## 🐛 Fix Navigation + RViz Glitch

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Then in `~/.bashrc`:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 📝 Edit Nav2 Parameters for TurtleBot3

```bash
cd /opt/ros/humble/share/turtlebot3_navigation2/param
sudo gedit waffle.yaml
```

Replace:

```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

---

## 🗺️ Load a Saved Map for Navigation

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/Documents/Nav/maps/my_map.yaml
```

---

## 🔀 Global & Local Planning Overview

* **Global Planner:** Uses costmap for long-range path planning.
* **Local Planner:** Short-term, reactive control.
* **Costmap Colors:** Red > Blue > White = Higher safety buffer

---

## 🔧 Dynamic Reconfigure with `rqt`

```bash
rqt
# Navigate to: Plugins > Configuration > Dynamic Reconfigure
```

Adjust `inflation_layer.inflation_radius` for tuning.

---

## 📐 Coordinate Frames with TF2

```bash
ros2 run tf2_tools view_frame
```

* `map`: Fixed world
* `odom`: Relative estimate
* `base_footprint`: Robot's base

---

## 🧩 Nav2 Architecture Components

* `map_server`: Handles map files
* `amcl`: Localization
* `planner_server`: Global planning
* `controller_server`: Local planning
* `recovery_server`: Error recovery
* `behavior_tree`: Decision logic
* `lifecycle_manager`: Manages node states

---

## 🛠️ Build Workspace

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git .
git checkout humble-devel
```

---

## 📄 URDF - Robot Description

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

Fix for xacro error:

```bash
mv my_robot.urdf my_robot.xacro
ros2 launch urdf_tutorial display.launch.py model:=/path/to/my_robot.xacro
```

---

## ⚙️ Odometry Setup

Use `ros2_control` with `diff_drive_controller` for odometry from wheel encoders.

---

## 🧭 SLAM Toolbox Workflow (Any Robot)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 run rviz2 rviz2
```

## 💾 Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Documents/Nav/maps/my_world06
```

## 📍 Navigate Using Saved Map

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=~/Documents/Nav/maps/my_world06.yaml
```

---

## 🎮 RViz2 Setup

* Add: TF, LaserScan, RobotModel, Map (Transient Local)
* Add Pose Estimate manually
* Add Global/Local Costmaps and Paths:

  * `/global_costmap/costmap` → Global
  * `/local_costmap/costmap` → Local
  * `/plan` and `/local_plan` → Paths

---

## ⚙️ Launch & Param Files

* **Launch Files:** Automate node startup

  * `/opt/ros/humble/share/turtlebot3_navigation2/launch/`
* **Parameter Files:** Node settings

  * `/opt/ros/humble/share/turtlebot3_navigation2/param/`

Usage:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=<map.yaml> params_file:=<waffle.yaml>
```

---

## 🧠 `nav2_simple_commander` API

A Python interface to interact with Nav2:

### Installation

```bash
sudo apt install ros-humble-nav2-simple-commander
sudo apt install ros-humble-tf-transformations
```

### Inspect Topics and Actions

```bash
ros2 topic info /initialpose
ros2 action info /navigate_to_pose
```

---

## 🧪 Example Python Scripts

### 1. Set Initial Pose

```python
# nav2_set_initial_pose.py
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

# ... [script as shown above] ...
```

### 2. Send Navigation Goal

```python
# nav2_send_goal.py
# ... [similar structure to set a goal pose and call goToPose()] ...
```

### 3. Waypoint Navigation

```python
# nav2_waypoints.py
# ... [create multiple poses and call followWaypoints()] ...
```

---

## 📸 Screenshots

![Screenshot 1](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/dbec009c-1577-403b-941a-90722be8a529/Screenshot_\(252\).png)
![Screenshot 2](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/961cf786-e8ad-4640-a93a-23eabc71eb31/Screenshot_\(253\).png)

---

## ✅ Done!

You are now ready to map, navigate, and command your robot using the Nav2 stack with ROS 2 Humble.

---

For contributions, improvements, or bug fixes, feel free to submit a PR or open an issue.
