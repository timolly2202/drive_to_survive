# 41118_project-sample

This repository contains the drive_to_survive project, demonstrating an autonomous driving workflow in Gazebo. It includes:

- A race track world and an “audibot” (Audi) vehicle model
- ROS 2 launch files for running Gazebo and RViz
- Scripts or nodes for controlling the robot, sensor data visualization, and basic autonomy
- Example Xacro and URDF definitions for the Audi with multiple sensors (LIDAR, RGB-D cameras, etc.)

The goal is to:

- Launch a simulated race track with an Audi vehicle
- Use sensor data (e.g. LIDAR, camera) to learn or follow the track
- Develop AI/ML strategies to optimize lap times or handle obstacles.

## Group information

Group 1: Drive to Survive

- 13886965 — Tejas Bhuta
- 13886965 — Jarred Deluca
- 13892512 — Tim Ollerton
- 13288232 — Wil Coxon

## Install info

This project requires ROS2 (Humble) with Gazebo and RViz, set up on Ubuntu 22.04 in order to run.

```bash
sudo apt install ros-$ROS_DISTRO-tf-transformations
pip install numpy==1.23.5 scikit-learn pandas joblib pytorch ultralytics cv-bridge
```

Once these dependencies are installed, clone the repository into your workspace and install rosdep:

1. **Clone the repository into your workspace:** 
```bash
git clone git@github.com:TejasB4/drive_to_survive.git
```

2. **Install Dependencies**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build Packages**
```bash
cd ~/drive_to_survive
colcon build --symlink-install
source install/setup.bash
```

## Run commands

Several different nodes are required to run the simulation. Run these commands in separate terminals.

```bash
ros2 launch gazebo_tf drive_to_survive.launch.py # launch the rviz simulation
ros2 launch audibot_yolo multi_camera.launch.py # launch the yolo camera nodes

```
