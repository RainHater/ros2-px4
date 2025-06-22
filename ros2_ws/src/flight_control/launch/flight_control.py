from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_offboard_node = Node(
        package='flight_control',
        executable='arming_offboard_node',
        name='arming_offboard_node',
        output='screen',
    )

    position_ctrl_node = Node(
        package='flight_control',
        executable='position_ctrl_node',
        name='position_ctrl_node',
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
        arm_offboard_node,
        position_ctrl_node,
        gps_navigation_node,
        rect_detect_node,
    ])
