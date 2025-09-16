from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    flight_mode_manager_node = Node(
        package='control_motion',
        executable='flight_mode_manager_node',
        name='flight_mode_manager_node',
        output='screen',
    )

    return LaunchDescription([
        flight_mode_manager_node
    ])
