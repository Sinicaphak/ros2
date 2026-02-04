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
        
        self.declare_parameter('pic_topic', '/car/pic')
        self.declare_parameter('fps', 30)
        self.declare_parameter('pic_dir', '/home/apollo/disk/ros2/src/car/pic/3')

        self.pic_topic = self.get_parameter('pic_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.pic_dir = self.get_parameter('pic_dir').get_parameter_value().string_value
        
        time.sleep(4)

        self.publisher_ = self.create_publisher(Image, self.pic_topic, 10)
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 1. 更改话题名称为 /car/pic
        self.publisher_ = self.create_publisher(Image, self.pic_topic, 10)
        
        # 2. 设置发布频率
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.bridge = CvBridge()
        self.cv_images = []
        self.current_image_index = 0

        # 3. 加载所有图片
        self.load_images()
        self.get_logger().info(f'发布到话题{self.pic_topic}, 频率: {self.fps} FPS')


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


    def timer_callback(self):
        if not self.cv_images:
            self.get_logger().warn('没有可发布的图片，请检查图片目录。', throttle_duration_sec=5)
            return

        if self.current_image_index >= len(self.cv_images):
            # 已经全部发布完毕
            return
        
        # 获取当前要发布的图片
        cv_image = self.cv_images[self.current_image_index]
        # 需要保存文件名列表
        image_file = self.image_files[self.current_image_index]
        
        self.get_logger().debug(f'正在发布图片索引: {self.current_image_index}')
        
        # 转换并发布图片
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = image_file
        self.publisher_.publish(ros_image)
        
        # 更新索引, 如果发布完所有图片，停止发布
        self.current_image_index = self.current_image_index + 1
        if self.current_image_index >= len(self.cv_images):
            self.get_logger().info(f'已发布完所有图片共{len(self.cv_images)}张，停止发布。')
            self.timer.cancel()
        

def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = ImagePublisherNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()