from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    flight_controller_interface = Node(
        package='flight_controller_interface',
        executable='px4_bridge_node',
        name='px4_bridge_node',
        output='screen',
    )

    return LaunchDescription([
        flight_controller_interface
    ])
