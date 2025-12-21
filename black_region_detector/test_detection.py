"""
测试黑色区域检测功能
可以单独测试检测功能，不需要连接机器人
"""
import cv2
import sys
from black_region_detector import BlackRegionDetector


def test_with_camera(threshold=48, min_area=100):
    """使用摄像头测试检测功能"""
    detector = BlackRegionDetector(threshold=threshold, min_area=min_area)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    print("按 'q' 键退出，按 '+' 增加阈值，按 '-' 减少阈值")
    print(f"当前阈值: {threshold}, 最小面积: {min_area}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测
        mask, regions = detector.detect_regions(frame)
        line_info = detector.detect_line(frame)
        
        # 绘制结果
        result = detector.draw_detection_result(frame, regions)
        
        # 显示线条信息
        if line_info:
            center_x, center_y, angle = line_info
            h, w = frame.shape[:2]
            image_center_x = w // 2
            
            cv2.circle(result, (center_x + image_center_x, center_y), 
                      10, (255, 255, 0), -1)
            cv2.line(result, 
                    (image_center_x, h),
                    (center_x + image_center_x, center_y),
                    (255, 255, 0), 2)
            
            cv2.putText(result, f"Offset: {center_x}px", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
            cv2.putText(result, f"Angle: {angle:.1f}deg", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
        
        # 创建组合显示
        h, w = result.shape[:2]
        display = np.zeros((h, w * 2, 3), dtype=np.uint8)
        display[:, :w] = result
        display[:, w:] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.line(display, (w, 0), (w, h), (255, 255, 255), 2)
        
        cv2.putText(display, "Original | Result", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 255), 2)
        cv2.putText(display, "Mask", 
                   (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 255), 2)
        
        cv2.imshow("Black Region Detection Test", display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('+') or key == ord('='):
            threshold = min(255, threshold + 5)
            detector.set_threshold(threshold)
            print(f"阈值: {threshold}")
        elif key == ord('-') or key == ord('_'):
            threshold = max(0, threshold - 5)
            detector.set_threshold(threshold)
            print(f"阈值: {threshold}")
    
    cap.release()
    cv2.destroyAllWindows()


def test_with_image(image_path, threshold=48, min_area=100):
    """使用图片测试检测功能"""
    import numpy as np
    
    detector = BlackRegionDetector(threshold=threshold, min_area=min_area)
    
    image = cv2.imread(image_path)
    if image is None:
        print(f"无法读取图片: {image_path}")
        return
    
    # 检测
    mask, regions = detector.detect_regions(image)
    line_info = detector.detect_line(image)
    
    # 绘制结果
    result = detector.draw_detection_result(image, regions)
    
    # 显示线条信息
    if line_info:
        center_x, center_y, angle = line_info
        h, w = image.shape[:2]
        image_center_x = w // 2
        
        cv2.circle(result, (center_x + image_center_x, center_y), 
                  10, (255, 255, 0), -1)
        cv2.line(result, 
                (image_center_x, h),
                (center_x + image_center_x, center_y),
                (255, 255, 0), 2)
        
        cv2.putText(result, f"Offset: {center_x}px", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (255, 255, 0), 2)
        cv2.putText(result, f"Angle: {angle:.1f}deg", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (255, 255, 0), 2)
    
    # 创建组合显示
    h, w = result.shape[:2]
    display = np.zeros((h, w * 2, 3), dtype=np.uint8)
    display[:, :w] = result
    display[:, w:] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.line(display, (w, 0), (w, h), (255, 255, 255), 2)
    
    cv2.putText(display, "Original | Result", 
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
               1, (0, 255, 255), 2)
    cv2.putText(display, "Mask", 
               (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
               1, (0, 255, 255), 2)
    
    cv2.imshow("Black Region Detection Test", display)
    print("按任意键退出...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import numpy as np
    
    if len(sys.argv) > 1:
        # 使用图片测试
        image_path = sys.argv[1]
        threshold = int(sys.argv[2]) if len(sys.argv) > 2 else 48
        min_area = int(sys.argv[3]) if len(sys.argv) > 3 else 100
        test_with_image(image_path, threshold, min_area)
    else:
        # 使用摄像头测试
        threshold = 48
        min_area = 100
        test_with_camera(threshold, min_area)


