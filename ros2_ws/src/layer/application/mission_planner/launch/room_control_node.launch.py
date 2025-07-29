from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mission_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mission_planner'),
                'launch',
                'mission_planner.launch.py'
            ])
        )
    )

    room_control_node = Node(
        package='mission_planner',
        executable='room_control_node',
        name='room_control_node',
        output='screen',
    )

    return LaunchDescription([
        mission_planner,
        room_control_node,
    ])
