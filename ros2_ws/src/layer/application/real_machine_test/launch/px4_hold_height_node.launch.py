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

    px4_hold_height_test_node = Node(
        package='real_machine_test',
        executable='px4_hold_height_test_node',
        name='px4_hold_height_test_node',
        output='screen',
    )
    return LaunchDescription([
        base_node,
        px4_hold_height_test_node,
    ])
