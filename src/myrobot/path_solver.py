#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('path_solver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # --- 地图配置 ---
        self.res = 0.05  # 分辨率
        self.size = 1.8
        self.width = int(self.size / self.res) + 1
        self.grid = np.zeros((self.width, self.width))
        
        # 安全间距：硬边距防止碰撞，软边距引导路径远离墙壁
        self.hard_margin = 0.12 # 略大于机器人半径 (0.15/2 = 0.075)
        self.soft_margin = 0.20 
        
        self.current_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.start_pos = None
        self.end_pos = (0.9, 1.6) # END 区域中心
        self.path = []
        self.target_idx = 0
        
        self.setup_map()
        self.get_logger().info("地图建模完成，等待 Odom 数据...")
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz 控制频率

    def add_wall(self, x1, y1, x2, y2):
        """ 将 URDF 中的墙体线段转化为网格占据点，并增加膨胀层 """
        steps = int(max(abs(x2-x1), abs(y2-y1)) / (self.res / 2)) + 1
        for s in range(steps):
            curr_x = x1 + (x2-x1) * s / steps
            curr_y = y1 + (y2-y1) * s / steps
            
            # 填充硬障碍 (999) 和 软势场 (增加代价值)
            mi = int(curr_x / self.res)
            mj = int(curr_y / self.res)
            
            # 膨胀半径内的网格处理
            r_idx = int(self.soft_margin / self.res)
            for i in range(mi - r_idx, mi + r_idx + 1):
                for j in range(mj - r_idx, mj + r_idx + 1):
                    if 0 <= i < self.width and 0 <= j < self.width:
                        dist = math.sqrt((i*self.res - curr_x)**2 + (j*self.res - curr_y)**2)
                        if dist < self.hard_margin:
                            self.grid[i, j] = 999
                        elif dist < self.soft_margin and self.grid[i, j] < 999:
                            self.grid[i, j] += 10 # 路径惩罚

    def setup_map(self):
        # 严格对应 maze.urdf 的墙体坐标
        walls = [
            (0, 0, 1.8, 0), (0, 1.8, 1.8, 1.8), (0, 0, 0, 1.8), (1.8, 0, 1.8, 1.8), # 外墙
            (0.45, 0, 0.45, 0.45), (0, 0.45, 0.45, 0.45), (0.45, 0.45, 0.45, 0.75), # GF, FE, M1F
            (0.8, 0.3, 1.8, 0.3), (0.45, 0.75, 1.45, 0.75), (0.0, 1.05, 0.6, 1.05), # LM, L1M1, KN
            (1.2, 1.15, 1.8, 1.15), (0.6, 1.4, 1.5, 1.4), (0.1, 1.25, 0.1, 1.7),   # N1K1, IH, ST
            (0.1, 1.7, 0.55, 1.7) # RS
        ]
        for w in walls: self.add_wall(w[0], w[1], w[2], w[3])

    def odom_callback(self, msg):
        # 获取位置
        pos = msg.pose.pose.position
        self.current_pos[0] = pos.x
        self.current_pos[1] = pos.y
        # 解析四元数获取 Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_pos[2] = math.atan2(siny_cosp, cosy_cosp)

        if self.start_pos is None:
            self.start_pos = (pos.x, pos.y)
            self.path = self.plan_path()
            if self.path: self.publish_path()

    def plan_path(self):
        start = (int(self.current_pos[0]/self.res), int(self.current_pos[1]/self.res))
        goal = (int(self.end_pos[0]/self.res), int(self.end_pos[1]/self.res))
        
        queue = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}

        while queue:
            current = heapq.heappop(queue)[1]
            if current == goal: break
            
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.width:
                    if self.grid[neighbor[0], neighbor[1]] >= 999: continue
                    
                    move_cost = math.sqrt(dx**2 + dy**2)
                    new_cost = cost_so_far[current] + move_cost + self.grid[neighbor[0], neighbor[1]]
                    
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + math.sqrt((neighbor[0]-goal[0])**2 + (neighbor[1]-goal[1])**2)
                        heapq.heappush(queue, (priority, neighbor))
                        came_from[neighbor] = current
        
        # 回溯路径并进行稀疏化（每隔2个点取一个点，减少控制抖动）
        full_path = []
        curr = goal
        while curr:
            full_path.append((curr[0]*self.res, curr[1]*self.res))
            curr = came_from.get(curr)
        return full_path[::-3] if len(full_path) > 5 else full_path[::-1]

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = "odom"
        for p in self.path:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = p[0], p[1]
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def control_loop(self):
        if not self.path or self.target_idx >= len(self.path):
            self.cmd_pub.publish(Twist())
            return

        target = self.path[self.target_idx]
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.08: # 到达当前路点
            self.target_idx += 1
            return

        # --- 核心改进：角度与速度控制 ---
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_pos[2]
        # 角度归一化 (-pi to pi)
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        msg = Twist()
        
        # 逻辑：如果角度偏差很大，先原地转弯；角度对准后再前进
        if abs(yaw_error) > 0.5: # 约 30度
            msg.angular.z = 0.8 if yaw_error > 0 else -0.8
            msg.linear.x = 0.0
        else:
            # PID 简易角度微调 + 前进
            msg.angular.z = 1.5 * yaw_error
            msg.linear.x = min(0.15, 0.5 * dist) # 速度限重与平滑减速
            
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
