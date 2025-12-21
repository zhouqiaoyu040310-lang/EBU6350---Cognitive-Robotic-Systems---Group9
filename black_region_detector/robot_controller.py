"""
机器人控制模块
支持串口通信和ROS通信两种方式控制真实机器人
"""
import time
import serial
import threading
from typing import Optional, Tuple
from enum import Enum


class MovementDirection(Enum):
    """运动方向枚举"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"


class RobotController:
    """机器人控制器"""
    
    def __init__(self, connection_type: str = "serial", 
                 port: str = "COM3", 
                 baudrate: int = 115200,
                 ros_topic: Optional[str] = None):
        """
        初始化机器人控制器
        
        Args:
            connection_type: 连接类型，"serial" 或 "ros"
            port: 串口端口（Windows: COM3, Linux: /dev/ttyUSB0）
            baudrate: 波特率
            ros_topic: ROS话题名称（如果使用ROS）
        """
        self.connection_type = connection_type
        self.port = port
        self.baudrate = baudrate
        self.ros_topic = ros_topic
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.current_speed = 0
        self.current_angle = 0
        
        # 速度参数
        self.max_speed = 100  # 最大速度（0-100）
        self.base_speed = 50  # 基础速度
        self.turn_speed = 30  # 转弯速度
        
        if connection_type == "serial":
            self._init_serial()
        elif connection_type == "ros":
            self._init_ros()
    
    def _init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)  # 等待串口稳定
            self.is_connected = True
            print(f"串口连接成功: {self.port} @ {self.baudrate}")
        except Exception as e:
            print(f"串口连接失败: {e}")
            self.is_connected = False
    
    def _init_ros(self):
        """初始化ROS连接"""
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import Twist
            
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = Node('robot_controller')
            self.cmd_vel_pub = self.ros_node.create_publisher(
                Twist, 
                self.ros_topic or '/cmd_vel', 
                10
            )
            self.is_connected = True
            print(f"ROS连接成功: {self.ros_topic or '/cmd_vel'}")
        except ImportError:
            print("警告: ROS2未安装，将使用模拟模式")
            self.is_connected = False
        except Exception as e:
            print(f"ROS连接失败: {e}")
            self.is_connected = False
    
    def send_command(self, command: str) -> bool:
        """
        发送命令到机器人
        
        Args:
            command: 命令字符串
            
        Returns:
            是否发送成功
        """
        if not self.is_connected:
            print(f"[模拟] 发送命令: {command}")
            return False
        
        try:
            if self.connection_type == "serial":
                if self.serial_conn and self.serial_conn.is_open:
                    command_bytes = (command + '\n').encode('utf-8')
                    self.serial_conn.write(command_bytes)
                    return True
            elif self.connection_type == "ros":
                # ROS命令通过Twist消息发送
                return self._send_ros_command(command)
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
        
        return False
    
    def _send_ros_command(self, command: str) -> bool:
        """发送ROS命令"""
        try:
            from geometry_msgs.msg import Twist
            msg = Twist()
            
            if command == "forward":
                msg.linear.x = self.base_speed / 100.0
            elif command == "backward":
                msg.linear.x = -self.base_speed / 100.0
            elif command == "left":
                msg.angular.z = self.turn_speed / 100.0
            elif command == "right":
                msg.angular.z = -self.turn_speed / 100.0
            elif command == "rotate_left":
                msg.angular.z = self.turn_speed / 100.0
            elif command == "rotate_right":
                msg.angular.z = -self.turn_speed / 100.0
            elif command == "stop":
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(msg)
            return True
        except Exception as e:
            print(f"ROS命令发送失败: {e}")
            return False
    
    def move_forward(self, speed: Optional[int] = None):
        """前进"""
        speed = speed or self.base_speed
        self.current_speed = speed
        if self.connection_type == "serial":
            self.send_command(f"MOVE_FORWARD:{speed}")
        else:
            self.send_command("forward")
    
    def move_backward(self, speed: Optional[int] = None):
        """后退"""
        speed = speed or self.base_speed
        self.current_speed = -speed
        if self.connection_type == "serial":
            self.send_command(f"MOVE_BACKWARD:{speed}")
        else:
            self.send_command("backward")
    
    def turn_left(self, speed: Optional[int] = None):
        """左转"""
        speed = speed or self.turn_speed
        self.current_angle = speed
        if self.connection_type == "serial":
            self.send_command(f"TURN_LEFT:{speed}")
        else:
            self.send_command("left")
    
    def turn_right(self, speed: Optional[int] = None):
        """右转"""
        speed = speed or self.turn_speed
        self.current_angle = -speed
        if self.connection_type == "serial":
            self.send_command(f"TURN_RIGHT:{speed}")
        else:
            self.send_command("right")
    
    def rotate_left(self, speed: Optional[int] = None):
        """原地左转"""
        speed = speed or self.turn_speed
        self.current_angle = speed
        if self.connection_type == "serial":
            self.send_command(f"ROTATE_LEFT:{speed}")
        else:
            self.send_command("rotate_left")
    
    def rotate_right(self, speed: Optional[int] = None):
        """原地右转"""
        speed = speed or self.turn_speed
        self.current_angle = -speed
        if self.connection_type == "serial":
            self.send_command(f"ROTATE_RIGHT:{speed}")
        else:
            self.send_command("rotate_right")
    
    def stop(self):
        """停止"""
        self.current_speed = 0
        self.current_angle = 0
        if self.connection_type == "serial":
            self.send_command("STOP")
        else:
            self.send_command("stop")
    
    def move_with_correction(self, line_offset: int, angle: float, 
                            max_offset: int = 100):
        """
        根据线条偏移量进行修正运动
        
        Args:
            line_offset: 线条中心相对于图像中心的偏移（像素）
            angle: 线条角度（度）
            max_offset: 最大允许偏移量（像素）
        """
        # 归一化偏移量到 [-1, 1]
        normalized_offset = line_offset / max_offset if max_offset > 0 else 0
        normalized_offset = max(-1, min(1, normalized_offset))
        
        # 如果偏移很小，直接前进
        if abs(normalized_offset) < 0.1:
            self.move_forward()
        # 如果线条偏左，向右修正
        elif normalized_offset < -0.1:
            # 偏移较大时，需要转弯
            if abs(normalized_offset) > 0.5:
                self.turn_right(int(self.turn_speed * abs(normalized_offset)))
            else:
                # 轻微修正，前进并右转
                self.move_forward()
                time.sleep(0.05)
                self.turn_right(int(self.turn_speed * abs(normalized_offset) * 0.5))
        # 如果线条偏右，向左修正
        else:
            if abs(normalized_offset) > 0.5:
                self.turn_left(int(self.turn_speed * abs(normalized_offset)))
            else:
                self.move_forward()
                time.sleep(0.05)
                self.turn_left(int(self.turn_speed * abs(normalized_offset) * 0.5))
    
    def disconnect(self):
        """断开连接"""
        self.stop()
        if self.connection_type == "serial" and self.serial_conn:
            if self.serial_conn.is_open:
                self.serial_conn.close()
            self.is_connected = False
        elif self.connection_type == "ros":
            try:
                import rclpy
                if rclpy.ok():
                    self.ros_node.destroy_node()
                    rclpy.shutdown()
            except:
                pass
            self.is_connected = False
        print("机器人连接已断开")


