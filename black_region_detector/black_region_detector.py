"""
黑色区域检测模块
用于检测图像中的黑色区域，支持线条跟踪和区域检测
"""
import cv2
import numpy as np
from typing import List, Tuple, Optional


class BlackRegionDetector:
    """黑色区域检测器"""
    
    def __init__(self, threshold: int = 48, min_area: int = 100):
        """
        初始化检测器
        
        Args:
            threshold: 灰度阈值，低于此值的像素被认为是黑色
            min_area: 最小区域面积，小于此面积的区域将被忽略
        """
        self.threshold = threshold
        self.min_area = min_area
        
    def detect_regions(self, image: np.ndarray) -> Tuple[np.ndarray, List[dict]]:
        """
        检测图像中的黑色区域
        
        Args:
            image: 输入图像（BGR格式）
            
        Returns:
            mask: 二值化掩码图像
            regions: 检测到的区域信息列表，每个区域包含：
                - 'contour': 轮廓
                - 'area': 面积
                - 'centroid': 质心坐标
                - 'bbox': 边界框 (x, y, w, h)
        """
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 二值化处理
        _, mask = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY_INV)
        
        # 形态学操作，去除噪声
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= self.min_area:
                # 计算质心
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0
                
                # 计算边界框
                x, y, w, h = cv2.boundingRect(contour)
                
                regions.append({
                    'contour': contour,
                    'area': area,
                    'centroid': (cx, cy),
                    'bbox': (x, y, w, h)
                })
        
        return mask, regions
    
    def detect_line(self, image: np.ndarray) -> Optional[Tuple[int, int, float]]:
        """
        检测黑色线条的中心位置和角度（用于路径跟踪）
        
        Args:
            image: 输入图像（BGR格式）
            
        Returns:
            (center_x, center_y, angle) 或 None（如果未检测到线条）
            center_x: 线条中心X坐标（相对于图像中心，负值表示偏左，正值表示偏右）
            center_y: 线条中心Y坐标
            angle: 线条角度（度）
        """
        mask, regions = self.detect_regions(image)
        
        if not regions:
            return None
        
        # 找到最大的区域（通常是路径线条）
        largest_region = max(regions, key=lambda r: r['area'])
        
        # 计算线条中心相对于图像中心的偏移
        h, w = image.shape[:2]
        image_center_x = w // 2
        center_x = largest_region['centroid'][0] - image_center_x
        center_y = largest_region['centroid'][1]
        
        # 计算线条角度
        contour = largest_region['contour']
        if len(contour) >= 5:
            # 使用最小外接矩形计算角度
            rect = cv2.minAreaRect(contour)
            angle = rect[2]
            # 标准化角度到 [-90, 90]
            if angle < -45:
                angle += 90
        else:
            angle = 0.0
        
        return (center_x, center_y, angle)
    
    def draw_detection_result(self, image: np.ndarray, regions: List[dict]) -> np.ndarray:
        """
        在图像上绘制检测结果
        
        Args:
            image: 原始图像
            regions: 检测到的区域列表
            
        Returns:
            绘制了检测结果的图像
        """
        result = image.copy()
        
        for region in regions:
            # 绘制边界框（红色）
            x, y, w, h = region['bbox']
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # 绘制轮廓（绿色）
            cv2.drawContours(result, [region['contour']], -1, (0, 255, 0), 2)
            
            # 绘制质心（蓝色圆点）
            cx, cy = region['centroid']
            cv2.circle(result, (cx, cy), 5, (255, 0, 0), -1)
            
            # 显示面积
            cv2.putText(result, f"Area: {region['area']}", 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (0, 255, 255), 1)
        
        # 显示检测到的区域数量
        cv2.putText(result, f"Black Regions: {len(regions)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 255), 2)
        cv2.putText(result, f"Threshold: {self.threshold}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 255), 2)
        
        return result
    
    def set_threshold(self, threshold: int):
        """设置检测阈值"""
        self.threshold = threshold
    
    def set_min_area(self, min_area: int):
        """设置最小区域面积"""
        self.min_area = min_area


