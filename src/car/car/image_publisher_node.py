import subprocess
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        # 模式：local 或 camera
        self.declare_parameter('mode', 'local')
        self.declare_parameter('pic_topic', '/car/pic')
        self.declare_parameter('fps', 30)
        self.declare_parameter('pic_dir', '/home/apollo/disk/ros2/src/car/pic/0')
        self.declare_parameter('camera_device', '/dev/video114')
        self.declare_parameter('frame_width', 0)
        self.declare_parameter('frame_height', 0)

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.pic_topic = self.get_parameter('pic_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.pic_dir = self.get_parameter('pic_dir').get_parameter_value().string_value
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value

        time.sleep(4)

        self.publisher_ = self.create_publisher(Image, self.pic_topic, 10)
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bridge = CvBridge()
        self.cv_images = []
        self.image_files = []
        self.current_image_index = 0
        self.cap = None  # camera capture handle

        if self.mode == 'local':
            self.load_images()
            self.get_logger().info(f'本地图片模式: 目录 {self.pic_dir}, {len(self.cv_images)} 张, {self.fps} FPS')
        else:
            self.init_camera()
            self.get_logger().info(f'摄像头模式: 设备 {self.camera_device}, {self.fps} FPS')

    def init_camera(self):
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头 {self.camera_device}')
            rclpy.shutdown()
            os._exit(1)
            return

    def load_images(self):
        """从 self.pic_dir 加载所有图片文件。"""
        if not os.path.isdir(self.pic_dir):
            self.get_logger().error(f"图片目录不存在: {self.pic_dir}")
            return
        
        image_files = sorted([f for f in os.listdir(self.pic_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
        if not image_files:
            self.get_logger().error(f"在目录 {self.pic_dir} 中没有找到图片文件。")
            return
        self.get_logger().info(f"目录 {self.pic_dir}下共有{len(image_files)} 张图片，正在加载...")
        self.image_files = image_files

        for image_file in image_files:
            image_path = os.path.join(self.pic_dir, image_file)
            image = cv2.imread(image_path)
            if image is not None:
                self.cv_images.append(image)
            else:
                self.get_logger().warn(f"无法读取图片: {image_path}")
        
        if not self.cv_images:
            self.get_logger().error("没有成功加载任何图片。")
            
    def publish_frame(self, cv_image, frame_id):
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = frame_id
        self.publisher_.publish(ros_image)

    def timer_callback(self):
        if self.mode == 'local':
            if not self.cv_images:
                self.get_logger().warn('没有可发布的图片，请检查图片目录。', throttle_duration_sec=5)
                return
            if self.current_image_index >= len(self.cv_images):
                return
            cv_image = self.cv_images[self.current_image_index]
            image_file = self.image_files[self.current_image_index]
            self.publish_frame(cv_image, image_file)
            self.current_image_index += 1
            if self.current_image_index >= len(self.cv_images):
                reload = True
                if reload:
                    self.get_logger().info(f'已发布完所有图片共{len(self.cv_images)}张，重新发布。')
                    self.current_image_index = 0
                else:
                    self.get_logger().info(f'已发布完所有图片共{len(self.cv_images)}张，停止发布。')
                    self.timer.cancel()
        else:
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn('摄像头未打开，无法发布。', throttle_duration_sec=5)
                rclpy.shutdown()
                os._exit(1)
                return
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('读取摄像头帧失败。', throttle_duration_sec=5)
                return
            self.publish_frame(frame, 'camera_frame')
        

def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = ImagePublisherNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()