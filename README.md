# 🛤️ Offboard Rail-Following Drone with Depth Obstacle Avoidance

This project implements a **PX4 + MAVROS-based autonomous drone** that performs:

- 🎯 **Rail tracking using a YOLOv8 vision model**
- 🛑 **Obstacle avoidance using a depth camera**
- 🛠️ Built with **ROS2 Humble**, **Gazebo**, and **PX4 SITL**

---

## 🚀 Key Features

| Feature                | Description                                                                 |
|------------------------|-----------------------------------------------------------------------------|
| YOLOv5 Detection       | Custom-trained YOLO model detects and segments rail tracks in real-time     |
| Depth Camera Avoidance | Gazebo-based simulated depth camera detects obstacles and avoids them       |
| MAVROS Integration     | Sends velocity and position setpoints for PX4 OFFBOARD control              |
| ROS2 Bridge            | Uses `ros_gz_bridge` to connect Gazebo sensor data with ROS2 topics         |

---

## 📁 Project Structure

```bash
ros2_ws/
└── src/
    └── rail_following_package
    ├── package.xml
    ├── rail_following_package
    │   ├── Models
    │   │   └── best.pt
    │   ├── __init__.py
    │   ├── mission_node.py
    │   └── yolo_detected_node.py
    ├── README.md
    ├── setup.cfg
    └── setup.py
```
---
## 🛠️ Requirements

- ROS 2 Humble
- PX4 v1.15
- Gazebo Harmoric
- MAVROS
---

## ▶️ Run Instructions
**1. Build Workspace**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
**2. Start Gazebo PX4 Simulation**
```bash
make px4_sitl gz_x500_mono_cam
```
**3. Run ROS_GZ bridge**
```bash
source ~/ros2_gz_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge ...
```
**4. Run MAVROS**
```bash
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@127.0.0.1:14557
```
**5. Launch the mission node**
```bash
ros2 run rail_following_package yolo_detected_node.py
ros2 run rail_following_package mission_node.py
```
---
## 🛸 Flight Logic Summary

- Drone takes off to TARGET_ALT = 10.0m in OFFBOARD mode.
- Uses YOLO to detect the red rail line, computes center offset + angle.
- If no obstacle detected ahead, cruise forward at constant speed.
- If depth image detects obstacle within OBS_THRESH = 7.0m, the drone:
  - Stops forward motion
  - Strafes left/right based on obstacle density
  - Resumes tracking once clear
---
👨‍💻 Developer
- Name: LEE YOONSU
- Contact: [GitHub](https://github.com/LEE-YOONSU)
- Project: offboard_rail_following_drone
