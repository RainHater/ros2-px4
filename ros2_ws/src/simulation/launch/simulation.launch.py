from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    pkg_gz = get_package_share_directory('gazebo_ros')

    model_path = os.path.join(pkg_sim, 'models')
    world_path = os.path.join(pkg_sim, 'worlds', 'landing_place.world')
    gazebo_launch_path = os.path.join(pkg_gz, 'launch', 'gazebo.launch.py')

    # 设置 GAZEBO_MODEL_PATH 环境变量
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # 设置 world 参数
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
    )

    # 加载 gazebo_ros 的启动文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    return LaunchDescription([
        set_gazebo_model_path,
        world_arg,
        gazebo
    ])
