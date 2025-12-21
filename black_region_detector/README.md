# 机器人导航系统

基于计算机视觉的真实机器人路径跟踪导航系统，支持黑色线条检测和自动路径跟踪。

## 功能特性

- ✅ **黑色区域检测**: 基于OpenCV的实时黑色区域/线条检测
- ✅ **路径跟踪**: 自动跟踪地面上的黑色线条路径
- ✅ **机器人控制**: 支持串口和ROS两种通信方式
- ✅ **实时显示**: 可视化检测结果和控制状态
- ✅ **可配置**: 通过YAML配置文件灵活调整参数

## 系统要求

- Python 3.8+
- 摄像头（USB摄像头或内置摄像头）
- 机器人硬件（支持串口或ROS通信）

## 安装

1. 克隆或下载项目到本地

2. 安装依赖：
```bash
pip install -r requirements.txt
```

3. （可选）如果使用ROS2，需要额外安装：
```bash
pip install rclpy geometry-msgs
```

## 使用方法

### 基本使用

```bash
python robot_navigation.py
```

### 命令行参数

```bash
python robot_navigation.py --help
```

常用参数：
- `--camera 0`: 指定摄像头索引（默认0）
- `--connection serial`: 连接类型（serial或ros）
- `--port COM3`: 串口端口（Windows: COM3, Linux: /dev/ttyUSB0）
- `--baudrate 115200`: 波特率
- `--threshold 48`: 黑色检测阈值
- `--min-area 100`: 最小区域面积
- `--no-display`: 不显示图像窗口

### 使用配置文件

1. 编辑 `config.yaml` 文件，设置摄像头、检测和机器人参数

2. 使用配置文件运行：
```python
from config_loader import ConfigLoader
from robot_navigation import RobotNavigation

config = ConfigLoader('config.yaml')

navigation = RobotNavigation(
    camera_index=config.get('camera.index'),
    connection_type=config.get('robot.connection_type'),
    port=config.get('robot.port'),
    baudrate=config.get('robot.baudrate'),
    threshold=config.get('detection.threshold'),
    min_area=config.get('detection.min_area')
)

navigation.run()
```

## 模块说明

### black_region_detector.py
黑色区域检测模块，提供：
- `BlackRegionDetector`: 检测图像中的黑色区域
- `detect_regions()`: 检测所有黑色区域
- `detect_line()`: 检测黑色线条（用于路径跟踪）
- `draw_detection_result()`: 绘制检测结果

### robot_controller.py
机器人控制模块，提供：
- `RobotController`: 机器人控制器
- 支持串口通信和ROS通信
- 提供前进、后退、转弯等基本运动控制
- `move_with_correction()`: 根据线条偏移自动修正运动

### robot_navigation.py
导航主程序，整合视觉检测和机器人控制：
- `RobotNavigation`: 导航系统主类
- 实时图像处理和控制
- 可视化显示检测结果

### config_loader.py
配置文件加载器，支持YAML格式配置

## 串口通信协议

如果使用串口通信，机器人需要支持以下命令格式：

- `MOVE_FORWARD:50` - 前进，速度50
- `MOVE_BACKWARD:50` - 后退，速度50
- `TURN_LEFT:30` - 左转，速度30
- `TURN_RIGHT:30` - 右转，速度30
- `ROTATE_LEFT:30` - 原地左转，速度30
- `ROTATE_RIGHT:30` - 原地右转，速度30
- `STOP` - 停止

## ROS通信

如果使用ROS2，机器人需要订阅 `/cmd_vel` 话题，接收 `geometry_msgs/Twist` 消息。

## 操作说明

运行程序后：
- 按 `q` 键：退出程序
- 按 `s` 键：暂停/恢复导航（需要修改代码实现）

## 参数调整

### 检测阈值 (threshold)
- 值越小，检测越敏感（可能误检）
- 值越大，检测越严格（可能漏检）
- 建议范围：30-80

### 最小区域面积 (min_area)
- 过滤掉太小的噪声区域
- 根据图像分辨率调整
- 建议范围：50-500

### 最大线条偏移 (max_line_offset)
- 控制机器人对线条偏移的敏感度
- 值越大，允许的偏移越大
- 建议范围：100-200

## 故障排除

1. **摄像头无法打开**
   - 检查摄像头是否被其他程序占用
   - 尝试不同的摄像头索引（--camera 1, 2, ...）

2. **串口连接失败**
   - 检查串口端口是否正确
   - 检查波特率是否匹配
   - 检查串口是否被其他程序占用

3. **检测效果不佳**
   - 调整检测阈值
   - 改善光照条件
   - 调整摄像头角度和位置

4. **机器人运动异常**
   - 检查串口通信是否正常
   - 检查机器人硬件连接
   - 调整运动速度参数

## 许可证

本项目仅供学习和研究使用。

## 作者

机器人导航系统


