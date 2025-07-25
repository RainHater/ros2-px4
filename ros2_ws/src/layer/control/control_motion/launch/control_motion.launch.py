from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motion_controller = Node(
        package='motion_controller',
        executable='motion_controller',
        name='motion_controller',
        output='screen',
    )

    return LaunchDescription([
        motion_controller
    ])
