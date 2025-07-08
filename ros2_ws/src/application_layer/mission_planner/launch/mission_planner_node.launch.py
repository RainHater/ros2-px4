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

    rect_detect_node = Node(
        package='vision_pipeline',
        executable='rect_detect_node',
        name='rect_detect_node',
        output='screen',
    )

    mission_planner_node = Node(
        package='mission_planner',
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
        base_node,
        rect_detect_node,
        mission_planner_node,
        pid_viewer_node,
    ])
