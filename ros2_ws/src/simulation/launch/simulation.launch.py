from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import subprocess
import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument('model', default_value='iris', description='PX4 模型名')
    world_arg = DeclareLaunchArgument('world', default_value='empty', description='Gazebo 世界名')

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')

    try:
        top_dir = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode().strip()
    except subprocess.CalledProcessError:
        raise RuntimeError("当前路径不在一个 Git 仓库中，无法获取顶层路径")

    sitl_script = f'{top_dir}/px4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh'
    px4_bin = f'{top_dir}/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4'
    px4_src = f'{top_dir}/px4/PX4-Autopilot'
    px4_build = f'{top_dir}/px4/PX4-Autopilot/build/px4_sitl_default'

    microxrce_agent_cmd = ['MicroXRCEAgent', 'udp4', '-p', '8888']

    return LaunchDescription([
        model_arg,
        world_arg,
        ExecuteProcess(
            cmd=microxrce_agent_cmd,
            output='screen',
            shell=False
        ),
        ExecuteProcess(
            cmd=[
                sitl_script,
                px4_bin,
                'none',
                model,
                world,
                px4_src,
                px4_build
            ],
            shell=True,
            output='screen'
        )
    ])
