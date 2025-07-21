from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    base_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mission_planner'),
                'launch',
                'base_node.launch.py'
            ])
        )
    )

    fly_relative_direction_action = Node(
        package='task_executor',
        executable='fly_relative_direction_action',
        name='fly_relative_direction_action',
        output='screen',
    )

    controlled_descent_action = Node(
        package='task_executor',
        executable='controlled_descent_action',
        name='controlled_descent_action',
        output='screen',
    )

    set_offboard_mode_server = Node(
        package='task_executor',
        executable='set_offboard_mode_action',
        name='set_offboard_mode_action',
        output='screen',
    )

    px4_hold_height_test_node = Node(
        package='real_machine_test',
        executable='px4_hold_height_test_node',
        name='px4_hold_height_test_node',
        output='screen',
    )
    return LaunchDescription([
        base_node,
        fly_relative_direction_action,
        controlled_descent_action,
        set_offboard_mode_server,
        px4_hold_height_test_node,
    ])
