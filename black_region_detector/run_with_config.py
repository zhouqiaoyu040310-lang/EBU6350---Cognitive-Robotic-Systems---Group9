"""
使用配置文件运行导航系统的示例脚本
"""
from config_loader import ConfigLoader
from robot_navigation import RobotNavigation


def main():
    """主函数"""
    # 加载配置
    config = ConfigLoader('config.yaml')
    
    # 创建导航系统
    navigation = RobotNavigation(
        camera_index=config.get('camera.index', 0),
        connection_type=config.get('robot.connection_type', 'serial'),
        port=config.get('robot.port', 'COM3'),
        baudrate=config.get('robot.baudrate', 115200),
        threshold=config.get('detection.threshold', 48),
        min_area=config.get('detection.min_area', 100),
        show_display=config.get('display.show_window', True)
    )
    
    # 设置导航参数
    navigation.max_line_offset = config.get('detection.max_line_offset', 150)
    navigation.update_rate = config.get('navigation.update_rate', 30)
    navigation.frame_skip = config.get('navigation.frame_skip', 1)
    
    # 设置机器人控制参数
    navigation.controller.max_speed = config.get('motion.max_speed', 100)
    navigation.controller.base_speed = config.get('motion.base_speed', 50)
    navigation.controller.turn_speed = config.get('motion.turn_speed', 30)
    
    # 运行导航
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


