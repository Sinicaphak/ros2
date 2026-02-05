from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import requests
import numpy as np
import base64
import os
import time
from datetime import datetime
import re
from openai import OpenAI

class VllmAskNode(Node):
    def __init__(self):
        super().__init__('vllm_ask_node')
        
        self.total_duration = 0.0
        self.request_count = 0
        
                
        self.declare_parameter('pic_topic', '')
        self.declare_parameter('process_pic_topic', '')
        self.declare_parameter('commd_topic', '')
        self.declare_parameter('api_url', '')
        self.declare_parameter('compression_quality', 0)
        self.declare_parameter('img_width', 0)
        self.declare_parameter('img_hight', 0)
        self.declare_parameter('max_tokens', 0)
        self.declare_parameter('system_prompt', '')
        self.declare_parameter('human_prompt', '')
        self.declare_parameter('gpt_prompt', '')

        # 获取参数并赋值为实例变量
        self.pic_topic = self.get_parameter('pic_topic').get_parameter_value().string_value
        self.process_pic_topic = self.get_parameter('process_pic_topic').get_parameter_value().string_value
        self.commd_topic = self.get_parameter('commd_topic').get_parameter_value().string_value
        self.api_url = self.get_parameter('api_url').get_parameter_value().string_value
        self.compression_quality = self.get_parameter('compression_quality').get_parameter_value().integer_value
        self.img_width = self.get_parameter('img_width').get_parameter_value().integer_value
        self.img_hight = self.get_parameter('img_hight').get_parameter_value().integer_value
        self.max_tokens = self.get_parameter('max_tokens').get_parameter_value().integer_value
        self.system_prompt = self.get_parameter('system_prompt').get_parameter_value().string_value
        self.human_prompt = self.get_parameter('human_prompt').get_parameter_value().string_value
        self.gpt_prompt = self.get_parameter('gpt_prompt').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            self.pic_topic,
            self.image_callback,
            100)
        self.point_publisher_ = self.create_publisher(Point, self.commd_topic, 10)
        self.process_img_publisher_ = self.create_publisher(Image, self.process_pic_topic, 10)
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        imgBase64, imgName = self.__process_image(msg)
        self.__showImg(imgBase64, imgName)
        
        start_time = time.time()
        
        respone = self.__send_sequential_request(imgBase64, imgName)

        duration = time.time() - start_time
        self.total_duration += duration
        self.request_count += 1
        avg_duration = self.total_duration / self.request_count
        self.get_logger().info(f"--> 响应 for {imgName} ({duration:.2f}s) 平均: {avg_duration:.2f}s: \n{respone}")
        
        point = self.__parse_point_from_response(respone)
        if point is not None:
            self.__publish_point(point)
        
    def __showImg(self, imgBase64, imgName):
        # 解码 base64 为 numpy 数组
        nparr = np.frombuffer(base64.b64decode(imgBase64.split(',')[1]), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        # 转换并发布图片
        ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = imgName
        self.process_img_publisher_.publish(ros_image)

    def __process_image(self, msg):
        # 1. 解析图片
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv2.resize(cv_image, (self.img_width, self.img_hight))
        # 2. 压缩为JPEG
        success, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.compression_quality])
        if not success:
            self.get_logger().error("图片压缩失败")
            return None, None
        # 3. 编码为base64
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        base64_image = f"data:image/jpeg;base64,{jpg_as_text}"
        # 4. 获取文件名
        image_name = msg.header.frame_id if msg.header.frame_id else ""
        return base64_image, image_name

    def __publish_point(self, point):
        if point is not None:
            self.point_publisher_.publish(point)
            
    def __send_request_doubao(self, imgBase64, imgName):
        client = OpenAI(
            api_key="sk-zctlhvenvuillirzxzhnjsiefvwkbechhvzxsrsmjsodpoqi",
            base_url="https://api.siliconflow.cn/v1"
        )

        response = client.chat.completions.create(
            model="Qwen/Qwen2.5-VL-72B-Instruct",
            messages=[
                {
                    "role": "system",
                    "content": [
                        {"type": "text", "text": self.system_prompt}
                    ]
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": imgBase64}},
                        {"type": "text", "text": self.user_prompt}
                    ]
                }
            ]
        )

        try:
            response = response.choices[0].message.content
            # result = response.json()
            self.get_logger().info(f"豆包响应 for {imgName}: {response}")
            return response
        except Exception as e:
            self.get_logger().error(f"豆包请求失败: {e}")
            return None
    
    def __send_sequential_request(self, imgBase64, imgName):
        headers = {"Content-Type": "application/json"}
        payload = {
            "model": "/app/model",
            "messages": [
                {
                    "role": "system",
                    "content": [
                        {"type": "text", "text": self.system_prompt}
                    ]
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": imgBase64}},
                        {"type": "text", "text": self.human_prompt}
                    ]
                },
                {
                    "role": "assistant",
                    "content": [
                        {"type": "text", "text": self.gpt_prompt}
                    ]
                }
            ],
            "max_tokens": self.max_tokens,
            "temperature": 0.0,
        }


        try:        
            response = requests.post(self.api_url, headers=headers, json=payload)
            response.raise_for_status()            
            response_data = response.json()
            assistant_message = response_data['choices'][0]['message']['content']

            return assistant_message
        except requests.exceptions.RequestException as e:
            self.get_logger().info(f"  -> 请求失败 for {imgName}: {e}")
            
    def __parse_point_from_response(self, response_text):
        point = Point()
        point.z = 0.0
        match = re.search(r'\[\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\]', response_text)
        if match:
            point.x = float(match.group(1))
            point.y = float(match.group(2))
            return point
        else:
            self.get_logger().warn("模型输出未找到坐标，默认丢弃")
            return None

def main(args=None):
    rclpy.init(args=args)
    vllm_ask_node = VllmAskNode()
    rclpy.spin(vllm_ask_node)
    vllm_ask_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



