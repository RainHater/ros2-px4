from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    flight_controller_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('flight_controller_interface'),
                'launch',
                'flight_controller_interface.launch.py'
            ])
        )
    )

    control_motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('control_motion'),
                'launch',
                'control_motion.launch.py'
            ])
        )
    )

    vision_test_node = Node(
        package='mission_planner',
        executable='vision_test_node',
        name='vision_test_node',
        output='screen',
    )

    return LaunchDescription([
        flight_controller_interface,
        control_motion,
        vision_test_node,
    ])
