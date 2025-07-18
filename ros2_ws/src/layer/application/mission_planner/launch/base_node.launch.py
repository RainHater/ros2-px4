from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    px4_bridge_node = Node(
        package='flight_controller_interface',
        executable='px4_bridge_node',
        name='px4_bridge_node',
        output='screen',
    )

    state_estimator_service = Node(
        package='state_estimator',
        executable='state_estimator_service',
        name='state_estimator_service',
        output='screen',
    )

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

    gps_nav_action = Node(
        package='task_executor',
        executable='gps_nav_action',
        name='gps_nav_action',
        output='screen',
    )

    return LaunchDescription([
        px4_bridge_node,
        state_estimator_service,
        flight_mode_manager_node,
        offboard_ctrl_node,
        gps_nav_action
    ])
