from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    control_motion = Node(
        package='control_motion',
        executable='control_motion',
        name='control_motion',
        output='screen',
    )

    return LaunchDescription([
        control_motion
    ])
