from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_control_follower',
            executable='trajectory_follower_node',
            output='screen'
        )
    ])
