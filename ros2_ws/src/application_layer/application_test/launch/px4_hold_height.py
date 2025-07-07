from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    px4_bridge_node = Node(
        package='flight_controller_interface',
        executable='px4_bridge_node',
        name='px4_bridge_node',
        output='screen',
    )

    state_estimator_node = Node(
        package='state_estimator',
        executable='state_estimator_node',
        name='state_estimator_node',
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

    gps_nav_node = Node(
        package='task_executor',
        executable='gps_nav_node',
        name='gps_nav_node',
        output='screen',
    )

    px4_hold_height_test_node = Node(
        package='application_test',
        executable='px4_hold_height_test_node',
        name='px4_hold_height_test_node',
        output='screen',
    )
    return LaunchDescription([
        px4_bridge_node,
        state_estimator_node,
        flight_mode_manager_node,
        offboard_ctrl_node,
        gps_nav_node,
        px4_hold_height_test_node,
    ])
