import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'myrobot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    maze_path = os.path.join(pkg_share, 'urdf/maze.urdf')
    robot_path = os.path.join(pkg_share, 'urdf/yahboom_car.urdf')

    return LaunchDescription([
        # 1. 启动 Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # 2. 生成迷宫
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'maze', '-file', maze_path, '-x', '0', '-y', '0', '-z', '0'],
            output='screen'),

        # 3. 生成小车
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'yahboomcar', '-file', robot_path, '-x', '1.6', '-y', '0.2', '-z', '0.1'],
            output='screen'),
    ])
