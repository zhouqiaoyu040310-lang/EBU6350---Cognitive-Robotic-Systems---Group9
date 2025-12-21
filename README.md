# EBU6350 - Cognitive Robotic Systems - Group 9 项目文档

## 1. 项目概述

本项目为 Yahboom 小车设计的综合导航系统，包含基于 ROS2/Gazebo 的迷宫仿真环境以及基于 OpenCV 的视觉寻迹系统。项目旨在通过 A* 算法实现全局路径规划，并通过计算机视觉实现实时的路径跟踪控制。

## 2. 环境要求

### 2.1 系统环境

- **操作系统**: Ubuntu 22.04 (推荐在 VMware 虚拟机运行)
- **机器人框架**: ROS2 (Humble/Foxy)
- **仿真平台**: Gazebo 11

### 2.2 依赖安装

**ROS2 Gazebo 插件:**

```bash
sudo apt install ros-${ROS_DISTRO}-gazebo-*
```

**Python 依赖:**

```bash
pip install -r requirements.txt # 包含 opencv-python, numpy, pyyaml 等
```

## 3. 项目结构与模块说明

```
yahboomcar_ws/
├── src/
│   └── myrobot/                    # 核心功能包
│       ├── path_solver.py          # A* 路径规划算法节点
│       ├── robot_navigation.py     # 视觉导航主程序
│       ├── black_region_detector.py # 视觉检测模块
│       ├── robot_controller.py     # 底层控制适配器 (Serial/ROS)
│       ├── config_loader.py        # 配置文件加载器
│       ├── launch/
│       │   └── bringup_model.launch.py # Gazebo 仿真启动脚本
│       └── urdf/                   # 模型文件 (包含迷宫与小车)
│           ├── maze.urdf
│           └── yahboom_car.urdf
└── config.yaml                     # 系统参数配置文件
```

### 3.1 仿真与路径规划模块

该模块用于在虚拟环境中验证算法逻辑。

- **A* 算法实现**: 节点订阅 `/odom` 获取位姿，通过 0.05m 分辨率的栅格地图规划路径。
- **碰撞预防**: 引入硬边距 (0.12m) 和软边距 (0.20m) 概念，确保路径远离墙壁中心。
- **控制频率**: 20Hz。

### 3.2 视觉检测与跟踪模块

该模块负责实时处理摄像头图像并驱动机器人。

- **检测逻辑**: 基于 OpenCV 实现，提取黑色线条/区域的质心偏移量。
- **纠偏控制**: `move_with_correction` 函数根据偏移量自动调整左右轮速比。
- **通信适配**: 支持通过 Serial 串口协议控制实体机器人，或通过 ROS 话题控制仿真模型。

## 4. 关键技术参数

### 4.1 迷宫环境参数

- **尺寸**: 1.8m × 1.8m
- **起点坐标**: (1.6, 0.2)
- **终点区域**: (0.9, 1.6) 绿色标记区

### 4.2 视觉检测参数 (config.yaml)

- `threshold`: 黑色检测阈值 (建议 30-80)
- `min_area`: 最小过滤面积 (建议 50-500)
- `max_line_offset`: 最大容许偏移量

## 5. 操作流程

### 5.1 编译工作空间

```bash
cd ~/yahboomcar_ws
colcon build --packages-select myrobot
source install/setup.bash
```

### 5.2 启动仿真模式

**启动环境:**

```bash
ros2 launch myrobot bringup_model.launch.py
```

**运行规划节点:**

```bash
python3 src/myrobot/path_solver.py
```

### 5.3 启动视觉导航模式

```bash
# 默认使用索引为 0 的摄像头，连接方式设为 ros 或 serial
python3 src/myrobot/robot_navigation.py --connection ros
```

## 6. 通信协议说明

### 6.1 ROS 接口

- **订阅**: `/odom` (nav_msgs/Odometry) - 获取实时位置。
- **发布**: `/cmd_vel` (geometry_msgs/Twist) - 发送运动指令。
- **可视化**: `/planned_path` (nav_msgs/Path) - Rviz 路径显示。

### 6.2 串口指令 (实体机器人)

当 `--connection serial` 时，系统通过以下指令与底层驱动板通信：

- `MOVE_FORWARD:速度`
- `TURN_LEFT:速度`
- `STOP`

## 7. 故障排除

- **Gazebo 模型不显示**: 检查 `~/.gazebo/models` 下是否完整下载了离线模型。
- **机器人原地震荡**: 减小 `path_solver.py` 中的 PID 增益或增加目标点的容差距离。
- **视觉检测失败**: 检查 `config.yaml` 中的阈值设置，确保环境光照不会导致画面过曝。

---

**开发信息**: EBU6350 Group 9 (Cognitive Robotic Systems)