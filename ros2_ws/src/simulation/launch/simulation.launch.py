import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    pkg_gz = get_package_share_directory('gazebo_ros')

    model_path = os.path.join(pkg_sim, 'models')
    world_path = os.path.join(pkg_sim, 'worlds', 'landing_place.world')
    gazebo_launch_path = os.path.join(pkg_gz, 'launch', 'gazebo.launch.py')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    return LaunchDescription([
        set_gazebo_model_path,
        world_arg,
        gazebo
    ])

