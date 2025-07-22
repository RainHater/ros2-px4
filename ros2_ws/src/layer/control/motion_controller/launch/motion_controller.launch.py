from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    flight_mode_manager_node = Node(
        package='motion_controller',
        executable='flight_mode_manager_node',
        name='flight_mode_manager_node',
        output='screen',
    )

    offboard_ctrl_node = Node(
        package='motion_controller',
        executable='offboard_ctrl_node',
        name='offboard_ctrl_node',
        output='screen',
    )

    return LaunchDescription([
        flight_mode_manager_node,
        offboard_ctrl_node,
    ])
