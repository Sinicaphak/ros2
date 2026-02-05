from launch import LaunchDescription
from launch_ros.actions import Node

# 图片目录
PIC_DIR = "/home/apollo/disk/ros2/src/car/pic/5"
# 话题
PIC_TOPIC = "/car/pic"
PROCESS_PIC_TOPIC = "/car/process_pic"
COMMD_TOPIC = "/goal_point"
mode="camera"
# 发布频率(fps)
FPS = 30
# 模型API
API_URL = "http://localhost:8002/v1/chat/completions"
# 图片压缩质量 (1-100, 越低压缩率越高)
COMPRESSION_QUALITY = 50
IMG_WIDTH=426
IMG_HIGHT=240
# 模型输出最大token数
MAX_TOKENS = 20
# prompt

SYSTEM_PROMPT = """""
You are an AI assistant for autonomous driving. Analyze the scene and reason through driving decisions carefully.
"""""

USER_PROMPT = """""
Choose the correct movement from the following options: Turn left, Stop, Turn right, Move forward.
Dont answer anything else.
"""

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='car',
        #     executable='vllm_ask',
        #     name='vllm_ask',
        #     parameters=[{
        #         'pic_topic': PIC_TOPIC,
        #         'process_pic_topic': PROCESS_PIC_TOPIC,
        #         'commd_topic': COMMD_TOPIC,
        #         'api_url': API_URL,
        #         'compression_quality': COMPRESSION_QUALITY,
        #         'img_width': IMG_WIDTH,
        #         'img_hight': IMG_HIGHT,
        #         'max_tokens': MAX_TOKENS,
        #         'system_prompt': SYSTEM_PROMPT,
        #         'user_prompt': USER_PROMPT,
        #     }],
        # ),
        Node(
            package='car',
            executable='image_publisher',
            name='image_publisher',
            parameters=[{
                'pic_topic': PIC_TOPIC,
                'fps': FPS,
                'pic_dir': PIC_DIR,
                'mode': mode,
            }],
        ),
    ])