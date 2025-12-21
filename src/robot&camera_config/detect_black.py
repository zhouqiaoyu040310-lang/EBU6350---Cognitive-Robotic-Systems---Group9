#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人摄像头黑色区域检测程序
功能：捕捉摄像头画面并标出画面中的黑色部分（纯黑）
使用ROS2订阅摄像头话题
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class BlackRegionDetector(Node):
    def __init__(self, image_topic='/espRos/esp32camera', black_threshold=48):
        """
        初始化黑色区域检测器
        
        参数:
            image_topic: ROS2图像话题名称，默认为 '/espRos/esp32camera'
            black_threshold: 黑色阈值（0-255），值越小越严格，默认30
        """
        super().__init__('black_region_detector')
        self.image_topic = image_topic
        self.black_threshold = black_threshold
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_lock = False
        
        # 订阅图像话题（先尝试CompressedImage，如果失败再尝试Image）
        self.compressed_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'订阅图像话题: {image_topic}')
        self.get_logger().info('等待接收图像数据...')
        
    def image_callback(self, msg):
        """图像话题回调函数"""
        try:
            # 尝试解码CompressedImage
            if isinstance(msg, CompressedImage):
                # 将JPEG压缩图像转换为OpenCV格式
                np_arr = np.frombuffer(msg.data, np.uint8)
                self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # 如果是Image消息，使用cv_bridge转换
                self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.current_frame is not None:
                self.frame_lock = True
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {e}')
            self.current_frame = None
    
    def detect_black_regions(self, frame):
        """
        检测图像中的黑色区域
        
        参数:
            frame: 输入图像帧（BGR格式）
        
        返回:
            mask: 黑色区域的二值掩码
            contours: 黑色区域的轮廓列表
        """
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 创建黑色区域的掩码（阈值处理）
        # 黑色区域：灰度值小于等于阈值
        _, mask = cv2.threshold(gray, self.black_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # 形态学操作，去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算：去除小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算：填充小孔洞
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return mask, contours
    
    def draw_detections(self, frame, contours, mask):
        """
        在图像上绘制检测到的黑色区域
        
        参数:
            frame: 原始图像帧
            contours: 轮廓列表
            mask: 掩码图像
        
        返回:
            result_frame: 绘制了标记的结果图像
        """
        result_frame = frame.copy()
        
        # 绘制所有黑色区域的轮廓
        for i, contour in enumerate(contours):
            # 计算轮廓面积，过滤太小的区域
            area = cv2.contourArea(contour)
            if area < 100:  # 忽略面积小于100像素的区域
                continue
            
            # 绘制轮廓（绿色）
            cv2.drawContours(result_frame, [contour], -1, (0, 255, 0), 2)
            
            # 计算边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 绘制边界框（红色）
            cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # 计算中心点
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # 绘制中心点（蓝色圆点）
                cv2.circle(result_frame, (cx, cy), 5, (255, 0, 0), -1)
                
                # 显示区域信息
                info_text = f"Area: {int(area)}"
                cv2.putText(result_frame, info_text, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 在左上角显示统计信息
        info = f"Black Regions: {len([c for c in contours if cv2.contourArea(c) >= 100])}"
        cv2.putText(result_frame, info, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 显示阈值信息
        threshold_text = f"Threshold: {self.black_threshold}"
        cv2.putText(result_frame, threshold_text, (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return result_frame
    
    def run(self):
        """运行主循环"""
        self.get_logger().info("开始检测黑色区域...")
        self.get_logger().info("按 'q' 键退出")
        self.get_logger().info("按 '+' 键增加阈值（更宽松）")
        self.get_logger().info("按 '-' 键减少阈值（更严格）")
        
        try:
            while rclpy.ok():
                # 处理ROS2回调
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 检查是否有新帧
                if self.current_frame is not None and self.frame_lock:
                    frame = self.current_frame.copy()
                    self.frame_lock = False
                    
                    # 检测黑色区域
                    mask, contours = self.detect_black_regions(frame)
                    
                    # 绘制检测结果
                    result_frame = self.draw_detections(frame, contours, mask)
                    
                    # 显示原始图像和掩码
                    # 创建组合图像：原始图像 | 结果图像 | 掩码
                    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    
                    # 调整图像大小以适应显示（如果太大）
                    max_width = 640
                    if frame.shape[1] > max_width:
                        scale = max_width / frame.shape[1]
                        new_width = int(frame.shape[1] * scale)
                        new_height = int(frame.shape[0] * scale)
                        frame = cv2.resize(frame, (new_width, new_height))
                        result_frame = cv2.resize(result_frame, (new_width, new_height))
                        mask_colored = cv2.resize(mask_colored, (new_width, new_height))
                    
                    combined = np.hstack([frame, result_frame, mask_colored])
                    
                    # 显示结果
                    cv2.imshow('Black Region Detection - Original | Result | Mask', combined)
                    
                    # 键盘控制
                    key = cv2.waitKey(1) & 0xFF
                    
                    if key == ord('q'):
                        self.get_logger().info("退出程序")
                        break
                    elif key == ord('+') or key == ord('='):
                        self.black_threshold = min(255, self.black_threshold + 5)
                        self.get_logger().info(f"阈值调整为: {self.black_threshold}")
                    elif key == ord('-') or key == ord('_'):
                        self.black_threshold = max(0, self.black_threshold - 5)
                        self.get_logger().info(f"阈值调整为: {self.black_threshold}")
        
        except KeyboardInterrupt:
            self.get_logger().info("\n程序被用户中断")
        
        except Exception as e:
            self.get_logger().error(f"发生错误: {e}")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        cv2.destroyAllWindows()
        self.get_logger().info("资源已释放")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='机器人摄像头黑色区域检测（ROS2版本）')
    parser.add_argument('--topic', type=str, default='/espRos/esp32camera',
                       help='ROS2图像话题名称（默认: /espRos/esp32camera）')
    parser.add_argument('--threshold', type=int, default=48,
                       help='黑色阈值 0-255（默认: 30，值越小越严格）')
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建检测器并运行
    detector = BlackRegionDetector(
        image_topic=args.topic,
        black_threshold=args.threshold
    )
    
    try:
        detector.run()
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


