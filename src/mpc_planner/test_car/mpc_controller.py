import rclpy.logging 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from .mpc_core import *

class MPCController(Node):
    def read_param(self):
        # 声明参数
        self.declare_parameter('T', 0.1)
        self.declare_parameter('N', 10.0)
        self.declare_parameter('linear_mpc.q_weights', [1.0, 1.0, 0.5])
        self.declare_parameter('linear_mpc.r_weights', [0.1, 0.1])
        self.declare_parameter('linear_mpc.u_upper', [1.0, 1.0])
        self.declare_parameter('linear_mpc.u_lower', [0.0, 0.0])
        q_values = self.get_parameter('linear_mpc.q_weights').value
        r_values = self.get_parameter('linear_mpc.r_weights').value
        u_upper = self.get_parameter('linear_mpc.u_upper').value
        u_lower = self.get_parameter('linear_mpc.u_lower').value
        
        if not q_values or not r_values or not u_upper or not u_lower:
            self.get_logger().error("mpc 参数为空，请检查参数文件或默认值。")
            raise ValueError("mpc 参数为空")
        # 获取参数
        T = self.get_parameter('T').get_parameter_value().double_value
        N = int(self.get_parameter('N').get_parameter_value().double_value)
        Q = np.diag(q_values)
        R = np.diag(r_values)
        u_lower = np.array([u_lower[0], u_lower[1]])
        u_upper = np.array([u_upper[0], u_upper[1]])
        
        mpc_config = LinearMPCConfig(
            T, N, Q, R, u_lower, u_upper
        )
        return mpc_config
    
    def __init__(self):
        super().__init__('mpc_controller')
        
        try:
            mpc_config = self.read_param()
            self.get_logger().info(f"read_param:{mpc_config}")
        except Exception as e:
            self.get_logger().error(f"Error reading parameters: {e}")
            raise
        
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_goal = self.create_subscription(Point, '/goal_point', self.goal_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 100)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.reference_path_pub = self.create_publisher(Path, '/reference_path', 10)

        self.current_pose = None
        
        # 初始时没有目标，等待 GoalSender 发送
        self.goals = [] 
        self.goal_index = 0
        self.goal_point = None
        self.reference_path = []
        self.actual_path = []
        self.frame_id = "odom"

        self.mpc_solver = NonlinearMPC(mpc_config)
        self.timer = self.create_timer(mpc_config.T, self.control_loop)
        self.get_logger().info(f"MPC Controller Node started. Waiting for goals on /goal_point...")

    def goal_callback(self, msg):
        new_goal = np.array([msg.x, msg.y])
        self.get_logger().info(f"Received new goal via topic: {new_goal}")
        
        # 将新目标加入列表
        self.goals.append(new_goal)
        self.reference_path.append(new_goal.tolist())
        
        # 如果当前没有目标（处于空闲状态），则立即激活这个新目标
        if self.goal_point is None:
            self.goal_index = len(self.goals) - 1
            self.goal_point = self.goals[self.goal_index]
            self.get_logger().info(f"Starting execution with goal: {self.goal_point}")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(ori)
        self.current_pose = np.array([pos.x, pos.y, yaw])

    def control_loop(self):
        if self.current_pose is None:
            return

        # 记录实际路径
        self.actual_path.append(self.current_pose[:2].tolist())
        self.publish_path(self.actual_path, self.actual_path_pub, self.frame_id)
        
        if self.reference_path:
            self.publish_path(self.reference_path, self.reference_path_pub, self.frame_id)

        # 如果没有目标，停车等待
        if self.goal_point is None:
            # 可以在这里发布 0 速度，确保安全
            # self.publish_vel(0.0, 0.0) 
            return

        dx = self.goal_point[0] - self.current_pose[0]
        dy = self.goal_point[1] - self.current_pose[1]
        dist = math.hypot(dx, dy)

        if dist < 0.1:
            self.publish_vel(0.0, 0.0)
            self.get_logger().info(f"Goal {self.goal_index+1} reached: {self.goal_point}")
            
            # 切换到下一个目标
            if self.goal_index + 1 < len(self.goals):
                self.goal_index += 1
                self.goal_point = self.goals[self.goal_index]
                self.get_logger().info(f"Switching to next goal: {self.goal_point}")
            else:
                self.get_logger().info("All goals in queue reached. Waiting for new goals...")
                self.goal_point = None
            return

        v_cmd, w_cmd = self.mpc_solver.solve(self.current_pose, self.goal_point, self.get_logger())
        
        self.get_logger().info(f"""
            vel: v={v_cmd:.2f}, w={w_cmd:.2f}
            Dist: {dist:.3f} m
            Goal: {self.goal_point}
        """, throttle_duration_sec=1.0)
        self.publish_vel(v_cmd, w_cmd)

    def publish_path(self, path_points, publisher, frame_id):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        for pt in path_points:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        publisher.publish(path_msg)

    def publish_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher_cmd.publish(msg)

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        return 0, 0, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MPCController()
        rclpy.spin(node)
        node.destroy_node()
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Exception in MPCController: {e}")
        else:
            rclpy.logging.get_logger("mpc_controller").error(f"Exception in MPCController: {e}")
    finally:
        rclpy.shutdown()