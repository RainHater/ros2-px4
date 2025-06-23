from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    flight_mode_manager_node = Node(
        package='flight_control',
        executable='flight_mode_manager_node',
        name='flight_mode_manager_node',
        output='screen',
    )

    offboard_ctrl_node = Node(
        package='flight_control',
        executable='offboard_ctrl_node',
        name='offboard_ctrl_node',
        output='screen',
    )

    gps_navigation_node = Node(
        package='flight_control',
        executable='gps_navigation_node',
        name='gps_navigation_node',
        output='screen',
    )

    rect_detect_node = Node(
        package='vision_pipeline',
        executable='rect_detect_node.py',
        name='rect_detect_node',
        output='screen',
    )

    return LaunchDescription([
        flight_mode_manager_node,
        offboard_ctrl_node,
        gps_navigation_node,
        rect_detect_node,
    ])
