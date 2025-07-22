from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vision_pipeline = Node(
        package='vision_pipeline',
        executable='vision_pipeline',
        name='vision_pipeline',
        output='screen',
    )

    return LaunchDescription([
        vision_pipeline
    ])
