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

    motion_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('motion_controller'),
                'launch',
                'motion_controller.launch.py'
            ])
        )
    )

    task_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('task_executor'),
                'launch',
                'task_executor.launch.py'
            ])
        )
    )

    px4_hold_height_test_node = Node(
        package='real_machine_test',
        executable='px4_hold_height_test_node',
        name='px4_hold_height_test_node',
        output='screen',
    )

    return LaunchDescription([
        flight_controller_interface,
        motion_controller,
        task_executor,
        px4_hold_height_test_node,
    ])
