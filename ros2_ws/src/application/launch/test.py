from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    px4_bridge_node = Node(
        package='interface',
        executable='px4_bridge_node',
        name='px4_bridge_node',
        output='screen',
    )

    state_estimator_node = Node(
        package='perception',
        executable='state_estimator_node',
        name='state_estimator_node',
        output='screen',
    )

    flight_mode_manager_node = Node(
        package='control',
        executable='flight_mode_manager_node',
        name='flight_mode_manager_node',
        output='screen',
    )

    offboard_ctrl_node = Node(
        package='control',
        executable='offboard_ctrl_node',
        name='offboard_ctrl_node',
        output='screen',
    )

    gps_nav_node = Node(
        package='control',
        executable='gps_nav_node',
        name='gps_nav_node',
        output='screen',
    )

    rect_detect_node = Node(
        package='vision_pipeline',
        executable='rect_detect_node.py',
        name='rect_detect_node',
        output='screen',
    )

    mission_planner_node = Node(
        package='application',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
    )

    pid_viewer_node = Node(
        package='debug',
        executable='pid_viewer_node',
        name='pid_viewer_node',
        output='screen',
    )

    return LaunchDescription([
        px4_bridge_node,
        state_estimator_node,
        flight_mode_manager_node,
        offboard_ctrl_node,
        gps_nav_node,
        rect_detect_node,
        mission_planner_node,
        pid_viewer_node,
    ])
