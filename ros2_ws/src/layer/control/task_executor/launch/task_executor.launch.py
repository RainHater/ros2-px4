from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controlled_descent_action = Node(
        package='task_executor',
        executable='controlled_descent_action',
        name='controlled_descent_action',
        output='screen',
    )

    fly_relative_direction_action = Node(
        package='task_executor',
        executable='fly_relative_direction_action',
        name='fly_relative_direction_action',
        output='screen',
    )

    gps_nav_action = Node(
        package='task_executor',
        executable='gps_nav_action',
        name='gps_nav_action',
        output='screen',
    )

    set_offboard_mode_action = Node(
        package='task_executor',
        executable='set_offboard_mode_action',
        name='set_offboard_mode_action',
        output='screen',
    )

    return LaunchDescription([
        controlled_descent_action,
        fly_relative_direction_action,
        gps_nav_action,
        set_offboard_mode_action
    ])
