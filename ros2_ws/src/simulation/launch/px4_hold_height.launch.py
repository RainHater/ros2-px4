from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    simulation_launch = PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('simulation'),
                    'launch',
                    'simulation.launch.py'
                ])
            )
            
    mission_planner_launch = PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('real_machine_test'),
                    'launch',
                    'px4_hold_height_node.launch.py'
                ])
            )

    return LaunchDescription([
        IncludeLaunchDescription(
            simulation_launch,
            launch_arguments={
                'model': 'iris_downward_depth_camera',
                'world': 'landing_place'
            }.items(),
        ),
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    mission_planner_launch,
                )
            ]
        )
    ])
