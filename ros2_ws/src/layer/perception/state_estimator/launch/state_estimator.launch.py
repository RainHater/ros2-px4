from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    state_estimator = Node(
        package='state_estimator',
        executable='state_estimator_service',
        name='state_estimator_service',
        output='screen',
    )

    return LaunchDescription([
        state_estimator
    ])
