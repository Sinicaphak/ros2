import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import os
import time

# 默认路径
DEFAULT_GOAL_FILE = "/home/apollo/disk/ros2/src/mpc/goal/goal.txt"

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.declare_parameter('goal_file', DEFAULT_GOAL_FILE)
        self.goal_file = self.get_parameter('goal_file').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(Point, '/goal_point', 10)
        
        self.get_logger().info(f'Goal Sender Started. Reading from: {self.goal_file}')
        
        # 创建一个定时器，等待 controller 上线后发送目标
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.goals_sent = False

    def load_goals_from_file(self, filename):
        goals = []
        if not os.path.exists(filename):
            self.get_logger().error(f"Goal file not found: {filename}")
            return goals
        
        with open(filename, "r") as f:
            for line in f:
                # 忽略空行、# 开头的注释行以及 // 开头的注释行
                if not line or line.strip() == "" or line.startswith("#") or line.startswith("//"):
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    try:
                        x, y = float(parts[0]), float(parts[1])
                        goals.append((x, y))
                    except ValueError:
                        continue
        return goals

    def timer_callback(self):
        if self.goals_sent:
            return

        # 检查是否有接收者 (Controller) 订阅了该话题
        # 避免发送过快导致 controller 还没启动就发丢了
        sub_count = self.publisher_.get_subscription_count()
        if sub_count > 0:
            self.get_logger().info(f'Detected {sub_count} subscriber(s). Sending goals...')
            goals = self.load_goals_from_file(self.goal_file)
            
            for i, (x, y) in enumerate(goals):
                msg = Point()
                msg.x = x
                msg.y = y
                msg.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(f'Sent Goal {i+1}: x={x}, y={y}')
                # 稍微延时，确保消息有序且不堵塞缓冲区
                time.sleep(0.1) 
            
            self.get_logger().info(f'All {len(goals)} goals sent.')
            self.goals_sent = True
            
            # 发送完毕后，可以选择关闭节点或者保持存活
            # fail-safe: 保持存活，方便观察日志
        else:
            self.get_logger().info('Waiting for subscriber (MPC Controller)...')

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()