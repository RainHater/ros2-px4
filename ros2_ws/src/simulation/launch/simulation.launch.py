from launch import LaunchDescription
from launch.actions import ExecuteProcess
import subprocess
import os

def generate_launch_description():
    try:
        top_dir = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode().strip()
    except subprocess.CalledProcessError:
        raise RuntimeError("当前路径不在一个 Git 仓库中，无法获取顶层路径")

    sitl_script = f'{top_dir}/px4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh'
    px4_bin = f'{top_dir}/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4'
    model = 'iris_downward_depth_camera'
    world = 'landing_place'
    px4_src = f'{top_dir}/px4/PX4-Autopilot'
    px4_build = f'{top_dir}/px4/PX4-Autopilot/build/px4_sitl_default'

    return LaunchDescription([
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
