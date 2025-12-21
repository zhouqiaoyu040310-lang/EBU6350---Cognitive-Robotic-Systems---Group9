# EBU6350 - Cognitive Robotic Systems - Group 9 Project Documentation

## 1. Project Overview

This project is a comprehensive navigation system designed for the Yahboom car, including a maze simulation environment based on ROS2/Gazebo and a visual tracking system based on OpenCV. The project aims to achieve global path planning through the A* algorithm and real-time path tracking control through computer vision.

## 2. Environment Requirements

### 2.1 System Environment

- **Operating System**: Ubuntu 22.04 (Recommended to run in VMware virtual machine)
- **Robot Framework**: ROS2 (Humble)
- **Simulation Platform**: Gazebo 11

### 2.2 Dependency Installation

**ROS2 Gazebo Plugins:**

```bash
sudo apt install ros-${ROS_DISTRO}-gazebo-*
```

**Python Dependencies:**

```bash
pip install -r requirements.txt # Includes opencv-python, numpy, pyyaml, etc.
```

## 3. Project Structure and Module Description

```
yahboomcar_ws/
├── src/
│   └── myrobot/                    # Core package
│       ├── path_solver.py          # A* path planning algorithm node
│       ├── robot_navigation.py     # Visual navigation main program
│       ├── black_region_detector.py # Visual detection module
│       ├── robot_controller.py     # Low-level control adapter (Serial/ROS)
│       ├── config_loader.py        # Configuration file loader
│       ├── launch/
│       │   └── bringup_model.launch.py # Gazebo simulation launch script
│       └── urdf/                   # Model files (including maze and car)
│           ├── maze.urdf
│           └── yahboom_car.urdf
└── config.yaml                     # System parameter configuration file
```

### 3.1 Simulation and Path Planning Module

This module is used to verify algorithm logic in a virtual environment.

- **A* Algorithm Implementation**: The node subscribes to `/odom` to obtain pose and plans paths through a grid map with 0.05m resolution.
- **Collision Prevention**: Introduces hard margin (0.12m) and soft margin (0.20m) concepts to ensure paths stay away from wall centers.
- **Control Frequency**: 20Hz.

### 3.2 Visual Detection and Tracking Module

This module is responsible for real-time processing of camera images and driving the robot.

- **Detection Logic**: Based on OpenCV implementation, extracts centroid offset of black lines/regions.
- **Correction Control**: The `move_with_correction` function automatically adjusts left-right wheel speed ratio based on offset.
- **Communication Adapter**: Supports controlling physical robots through Serial port protocol, or controlling simulation models through ROS topics.

## 4. Key Technical Parameters

### 4.1 Maze Environment Parameters

- **Size**: 1.8m × 1.8m
- **Start Coordinates**: (1.6, 0.2)
- **End Zone**: (0.9, 1.6) green marked area

### 4.2 Visual Detection Parameters (config.yaml)

- `threshold`: Black detection threshold (recommended 30-80)
- `min_area`: Minimum filtering area (recommended 50-500)
- `max_line_offset`: Maximum allowable offset

## 5. Operation Procedures

### 5.1 Build Workspace

```bash
cd ~/yahboomcar_ws
colcon build --packages-select myrobot
source install/setup.bash
```

### 5.2 Launch Simulation Mode

**Launch Environment:**

```bash
ros2 launch myrobot bringup_model.launch.py
```

**Run Planning Node:**

```bash
python3 src/myrobot/path_solver.py
```

### 5.3 Launch Visual Navigation Mode

```bash
# Default uses camera with index 0, connection mode set to ros or serial
python3 src/myrobot/robot_navigation.py --connection ros
```

## 6. Communication Protocol Description

### 6.1 ROS Interface

- **Subscribe**: `/odom` (nav_msgs/Odometry) - Get real-time position.
- **Publish**: `/cmd_vel` (geometry_msgs/Twist) - Send motion commands.
- **Visualization**: `/planned_path` (nav_msgs/Path) - Rviz path display.

### 6.2 Serial Commands (Physical Robot)

When `--connection serial`, the system communicates with the underlying driver board through the following commands:

- `MOVE_FORWARD:speed`
- `TURN_LEFT:speed`
- `STOP`

## 7. Troubleshooting

- **Gazebo models not displaying**: Check if offline models are fully downloaded in `~/.gazebo/models`.
- **Robot oscillating in place**: Reduce PID gains in `path_solver.py` or increase tolerance distance of target points.
- **Visual detection failure**: Check threshold settings in `config.yaml` to ensure ambient lighting does not cause overexposure.

---

**Development Information**: EBU6350 Group 9 (Cognitive Robotic Systems)
