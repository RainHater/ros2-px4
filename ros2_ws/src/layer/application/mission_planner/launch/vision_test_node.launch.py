from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mipi_detect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mipi_detect'),
                'launch',
                'detect.launch.py'
            ])
        )
    )

    mission_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mission_planner'),
                'launch',
                'mission_planner.launch.py'
            ])
        )
    )

    vision_test_node = Node(
        package='mission_planner',
        executable='vision_test_node',
        name='vision_test_node',
        output='screen'
    )

    return LaunchDescription([
        mipi_detect,
        mission_planner,
        vision_test_node,
    ])
