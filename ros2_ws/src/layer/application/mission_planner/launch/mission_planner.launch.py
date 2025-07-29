from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

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

    return LaunchDescription([
        flight_controller_interface,
        control_motion
    ])
