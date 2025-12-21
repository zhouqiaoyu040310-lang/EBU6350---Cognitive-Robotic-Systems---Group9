"""
机器人导航主程序
整合视觉检测和机器人控制，实现真实的路径跟踪导航
"""
import cv2
import time
import numpy as np
from typing import Optional, Tuple
from black_region_detector import BlackRegionDetector
from robot_controller import RobotController


class RobotNavigation:
    """机器人导航系统"""
    
    def __init__(self, 
                 camera_index: int = 0,
                 connection_type: str = "serial",
                 port: str = "COM3",
                 baudrate: int = 115200,
                 threshold: int = 48,
                 min_area: int = 100,
                 show_display: bool = True):
        """
        初始化导航系统
        
        Args:
            camera_index: 摄像头索引（0为默认摄像头）
            connection_type: 连接类型，"serial" 或 "ros"
            port: 串口端口
            baudrate: 波特率
            threshold: 黑色检测阈值
            min_area: 最小区域面积
            show_display: 是否显示图像窗口
        """
        # 初始化检测器
        self.detector = BlackRegionDetector(threshold=threshold, min_area=min_area)
        
        # 初始化机器人控制器
        self.controller = RobotController(
            connection_type=connection_type,
            port=port,
            baudrate=baudrate
        )
        
        # 初始化摄像头
        self.camera_index = camera_index
        self.cap: Optional[cv2.VideoCapture] = None
        self.show_display = show_display
        
        # 导航参数
        self.running = False
        self.max_line_offset = 150  # 最大线条偏移量（像素）
        self.update_rate = 30  # 控制更新频率（Hz）
        self.frame_skip = 1  # 跳帧数（用于降低计算负载）
        self.frame_count = 0
        
        # 统计信息
        self.fps = 0
        self.last_time = time.time()
        self.frame_times = []
    
    def initialize_camera(self) -> bool:
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                print(f"无法打开摄像头 {self.camera_index}")
                return False
            
            # 设置摄像头参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            print(f"摄像头初始化成功: {self.camera_index}")
            return True
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            return False
    
    def get_frame(self) -> Optional[np.ndarray]:
        """获取一帧图像"""
        if self.cap is None:
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        return frame
    
    def process_frame(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int, float]], np.ndarray, np.ndarray]:
        """
        处理一帧图像
        
        Returns:
            (line_info, result_image, mask): 
            - line_info: (center_x, center_y, angle) 或 None
            - result_image: 绘制了检测结果的图像
            - mask: 二值化掩码
        """
        # 检测黑色区域
        mask, regions = self.detector.detect_regions(frame)
        
        # 检测线条
        line_info = self.detector.detect_line(frame)
        
        # 绘制检测结果
        result_image = self.detector.draw_detection_result(frame, regions)
        
        # 如果检测到线条，绘制导航信息
        if line_info:
            center_x, center_y, angle = line_info
            h, w = frame.shape[:2]
            image_center_x = w // 2
            
            # 绘制线条中心点
            cv2.circle(result_image, (center_x + image_center_x, center_y), 
                      10, (255, 255, 0), -1)
            
            # 绘制从图像中心到线条中心的线
            cv2.line(result_image, 
                    (image_center_x, h),
                    (center_x + image_center_x, center_y),
                    (255, 255, 0), 2)
            
            # 显示偏移信息
            cv2.putText(result_image, f"Offset: {center_x}px", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
            cv2.putText(result_image, f"Angle: {angle:.1f}deg", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
        
        return line_info, result_image, mask
    
    def control_robot(self, line_info: Optional[Tuple[int, int, float]]):
        """根据检测结果控制机器人"""
        if line_info is None:
            # 未检测到线条，停止或搜索
            self.controller.stop()
            print("未检测到线条，停止运动")
            return
        
        center_x, center_y, angle = line_info
        
        # 根据线条偏移量控制机器人
        self.controller.move_with_correction(
            center_x, 
            angle, 
            self.max_line_offset
        )
    
    def run(self):
        """运行导航主循环"""
        if not self.initialize_camera():
            print("摄像头初始化失败，无法启动导航")
            return
        
        if not self.controller.is_connected:
            print("警告: 机器人未连接，将使用模拟模式")
        
        self.running = True
        print("导航系统启动...")
        print("按 'q' 键退出，按 's' 键停止/启动")
        
        try:
            while self.running:
                # 计算FPS
                current_time = time.time()
                self.frame_times.append(current_time)
                if len(self.frame_times) > 30:
                    self.frame_times.pop(0)
                if len(self.frame_times) > 1:
                    self.fps = len(self.frame_times) / (self.frame_times[-1] - self.frame_times[0])
                
                # 获取帧
                frame = self.get_frame()
                if frame is None:
                    print("无法获取图像帧")
                    time.sleep(0.1)
                    continue
                
                # 跳帧处理
                self.frame_count += 1
                if self.frame_count % (self.frame_skip + 1) != 0:
                    continue
                
                # 处理帧
                line_info, result_image, mask = self.process_frame(frame)
                
                # 控制机器人
                self.control_robot(line_info)
                
                # 显示结果
                if self.show_display:
                    # 显示FPS
                    cv2.putText(result_image, f"FPS: {self.fps:.1f}", 
                               (10, result_image.shape[0] - 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 
                               0.7, (0, 255, 0), 2)
                    
                    # 显示状态
                    status = "运行中" if self.running else "已停止"
                    cv2.putText(result_image, f"Status: {status}", 
                               (10, result_image.shape[0] - 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 
                               0.7, (0, 255, 0), 2)
                    
                    # 创建组合显示
                    h, w = result_image.shape[:2]
                    display = np.zeros((h, w * 2, 3), dtype=np.uint8)
                    display[:, :w] = result_image
                    display[:, w:] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    
                    # 添加分隔线
                    cv2.line(display, (w, 0), (w, h), (255, 255, 255), 2)
                    
                    # 添加标签
                    cv2.putText(display, "Original | Result", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               1, (0, 255, 255), 2)
                    cv2.putText(display, "Mask", 
                               (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               1, (0, 255, 255), 2)
                    
                    cv2.imshow("Black Region Detection - Original | Result | Mask", display)
                    
                    # 处理按键
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.running = False
                        break
                    elif key == ord('s'):
                        # 切换运行状态
                        if self.running:
                            self.controller.stop()
                            print("导航已暂停")
                        else:
                            print("导航已恢复")
                        # 注意：这里需要外部控制running状态
                
                # 控制更新频率
                time.sleep(1.0 / self.update_rate)
        
        except KeyboardInterrupt:
            print("\n收到中断信号，正在停止...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("正在清理资源...")
        self.running = False
        self.controller.stop()
        self.controller.disconnect()
        
        if self.cap is not None:
            self.cap.release()
        
        if self.show_display:
            cv2.destroyAllWindows()
        
        print("清理完成")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='机器人导航系统')
    parser.add_argument('--camera', type=int, default=0, help='摄像头索引')
    parser.add_argument('--connection', type=str, default='serial', 
                       choices=['serial', 'ros'], help='连接类型')
    parser.add_argument('--port', type=str, default='COM3', help='串口端口')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--threshold', type=int, default=48, help='检测阈值')
    parser.add_argument('--min-area', type=int, default=100, help='最小区域面积')
    parser.add_argument('--no-display', action='store_true', help='不显示图像窗口')
    
    args = parser.parse_args()
    
    # 创建导航系统
    navigation = RobotNavigation(
        camera_index=args.camera,
        connection_type=args.connection,
        port=args.port,
        baudrate=args.baudrate,
        threshold=args.threshold,
        min_area=args.min_area,
        show_display=not args.no_display
    )
    
    # 运行导航
    navigation.run()


if __name__ == "__main__":
    main()


