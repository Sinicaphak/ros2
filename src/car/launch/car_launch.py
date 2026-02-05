from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.actions import Node

# 图片目录
PIC_DIR = "/home/apollo/disk/ros2/src/car/pic/5"
# 话题
PIC_TOPIC = "/car/pic"
PROCESS_PIC_TOPIC = "/car/process_pic"
COMMD_TOPIC = "/goal_point"
MODE="camera"
# 发布频率(fps)
FPS = 60
# 模型API
API_URL = "http://localhost:8002/v1/chat/completions"
# 图片压缩质量 (1-100, 越低压缩率越高)
COMPRESSION_QUALITY = 50
IMG_WIDTH=426
IMG_HIGHT=240
# 模型输出最大token数
MAX_TOKENS = 200
# prompt

SYSTEM_PROMPT = """""
You are an autonomous driving planner.
Coordinate system: X-axis is lateral, Y-axis is longitudinal.
The ego vehicle is at (0,0), units are meters.
Based on the provided front-view image and driving context, plan future waypoints at 0.5-second intervals for the next 3 seconds
"""""

HUMAN_PROMPT = """""
Here is the front-view image from the car.
Historical trajectory (last 2 seconds): [(0.00,-0.00), (0.00,-0.00), (0.00,-0.00), (0.00,-0.00)]
Mission goal: FORWARD
Traffic rules:
- Avoid collision with other objects.
- Always drive on drivable regions.
- Avoid occupied regions.

Please plan future waypoints at 0.5-second intervals for the next 3 seconds.
You MUST only output the trajectory within the specified format that system request.
"""

GPT_PROMPT = """""
Trajectory:
[(-0.00,0.00), (-0.00,0.00), (-0.00,0.00), (-0.00,0.00), (-0.00,0.00), (-0.00,0.00)]
"""

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='Logger level for all nodes')

    log_level = LaunchConfiguration('log_level')
    
    return LaunchDescription([
        Node(
            package='car',
            executable='vllm_ask',
            name='vllm_ask',
            parameters=[{
                'pic_topic': PIC_TOPIC,
                'process_pic_topic': PROCESS_PIC_TOPIC,
                'commd_topic': COMMD_TOPIC,
                'api_url': API_URL,
                'compression_quality': COMPRESSION_QUALITY,
                'img_width': IMG_WIDTH,
                'img_hight': IMG_HIGHT,
                'max_tokens': MAX_TOKENS,
                'system_prompt': SYSTEM_PROMPT,
                'human_prompt': HUMAN_PROMPT,
                'gpt_prompt': GPT_PROMPT,
                'camera_device': '/dev/video1'
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='car',
            executable='image_publisher',
            name='image_publisher',
            parameters=[{
                'pic_topic': PIC_TOPIC,
                'fps': FPS,
                'pic_dir': PIC_DIR,
                'mode': MODE,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])