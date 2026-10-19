from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动图像发布者
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher',
            # 如果发布的话题不是 /sensor_img，这里进行重映射
            remappings=[('/image_raw', '/sensor_img')] 
        ),
        # 启动你的检测节点
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen'
        )
    ])
