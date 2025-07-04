import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    pkg_gz = get_package_share_directory('gazebo_ros')

    model_path = os.path.join(pkg_sim, 'models')
    world_path = os.path.join(pkg_sim, 'worlds', 'landing_place.world')
    model_sdf_path = os.path.join(pkg_sim, 'models', 'iris_downward_depth_camera', 'iris_downward_depth_camera.sdf')
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

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'iris_drone_1',
                   '-file', model_sdf_path,
                   '-x', '1.0', '-y', '2.0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        world_arg,
        gazebo,
        spawn_model,
    ])

