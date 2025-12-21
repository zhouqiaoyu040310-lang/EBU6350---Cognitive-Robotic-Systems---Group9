"""
快速启动脚本
简化版的导航系统启动脚本
"""
from robot_navigation import RobotNavigation


def main():
    """快速启动导航系统"""
    print("=" * 50)
    print("机器人导航系统 - 快速启动")
    print("=" * 50)
    print()
    
    # 获取用户输入
    print("请选择连接方式:")
    print("1. 串口通信 (Serial)")
    print("2. ROS通信 (ROS)")
    print("3. 仅测试检测功能（不连接机器人）")
    
    choice = input("请输入选项 (1/2/3，默认1): ").strip() or "1"
    
    if choice == "3":
        # 仅测试检测
        print("\n启动检测测试模式...")
        from test_detection import test_with_camera
        test_with_camera()
        return
    
    # 获取连接参数
    if choice == "1":
        connection_type = "serial"
        port = input("请输入串口端口 (默认 COM3): ").strip() or "COM3"
        baudrate = input("请输入波特率 (默认 115200): ").strip() or "115200"
        try:
            baudrate = int(baudrate)
        except:
            baudrate = 115200
    else:
        connection_type = "ros"
        port = "N/A"
        baudrate = 115200
    
    # 获取摄像头索引
    camera_index = input("请输入摄像头索引 (默认 0): ").strip() or "0"
    try:
        camera_index = int(camera_index)
    except:
        camera_index = 0
    
    # 获取检测参数
    threshold = input("请输入检测阈值 (默认 48): ").strip() or "48"
    try:
        threshold = int(threshold)
    except:
        threshold = 48
    
    min_area = input("请输入最小区域面积 (默认 100): ").strip() or "100"
    try:
        min_area = int(min_area)
    except:
        min_area = 100
    
    print()
    print("=" * 50)
    print("配置信息:")
    print(f"  连接方式: {connection_type}")
    if connection_type == "serial":
        print(f"  串口端口: {port}")
        print(f"  波特率: {baudrate}")
    print(f"  摄像头索引: {camera_index}")
    print(f"  检测阈值: {threshold}")
    print(f"  最小区域面积: {min_area}")
    print("=" * 50)
    print()
    
    input("按回车键开始运行...")
    
    # 创建并运行导航系统
    navigation = RobotNavigation(
        camera_index=camera_index,
        connection_type=connection_type,
        port=port,
        baudrate=baudrate,
        threshold=threshold,
        min_area=min_area,
        show_display=True
    )
    
    try:
        navigation.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"运行错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()


