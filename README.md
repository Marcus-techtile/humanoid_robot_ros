# G1 Humanoid Robot ROS Packages

This repository contains ROS Noetic packages for simulating and controlling the Unitree G1 humanoid robot using MuJoCo physics engine.

## Package Structure

- **g1_description**: Robot model files (MJCF) and configuration
- **g1_mujoco_sim**: MuJoCo simulation interface for ROS
- **g1_control**: Control algorithms (standing state controller)
- **g1_bringup**: Launch files to bring up the complete system

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- MuJoCo (will be installed via pip)

## Installation

1. Install ROS Noetic dependencies:
```bash
sudo apt update
sudo apt install ros-noetic-robot-state-publisher ros-noetic-rviz ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs
```

2. Install Python dependencies:
```bash
pip3 install -r requirements.txt
```

3. Build the workspace:
```bash
cd ~/work_space_dir
catkin_make
source devel/setup.bash
```

## Running
To launch the G1 robot with MuJoCo visualization and ROS with visualization on RVIZ

```bash
roslaunch g1_bringup g1_standing_demo.launch
```

This will:
- Start the MuJoCo physics simulation with GUI using scene_mjx.xml
- Load the G1 robot model in ros and visualiza on rviz
- Launch the robot controller
- Joints can be controlled using the MuJoCo GUI
- Robot states on ROS are synced with MuJuCo

