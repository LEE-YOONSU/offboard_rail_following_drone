# Rail Following Drone with Obstacle Avoidance

This ROS 2 package enables a drone to follow a rail line using a downward mono camera and avoid obstacles using a forward depth camera, leveraging MAVROS and PX4 in OFFBOARD mode.

## Features
- Road line following via image processing
- Obstacle avoidance via depth image
- OFFBOARD position control using MAVROS

## Requirements
- ROS 2 Humble
- MAVROS
- PX4 SITL
- Gazebo with custom model `x500_mono_cam`

## Launch
```bash
ros2 run rail_following_package depth_line_follower
