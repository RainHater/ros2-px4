from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

def generate_launch_description():
    control_motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('control_motion'),
                'launch',
                'control_motion.launch.py'
            ])
        ),
    )

    return LaunchDescription([
        control_motion
    ])
