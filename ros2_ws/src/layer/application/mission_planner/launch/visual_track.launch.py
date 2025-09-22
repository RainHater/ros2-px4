from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    use_mipi_detect_arg = DeclareLaunchArgument(
        "use_mipi_detect",
        default_value="true",
        description="Whether to launch mipi_detect"
    )

    # mipi_detect launch
    mipi_detect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mipi_detect'),
                'launch',
                'detect.launch.py'
            ])
        )
    )

    mission_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mission_planner'),
                'launch',
                'mission_planner.launch.py'
            ])
        )
    )

    visual_track_node = Node(
        package='mission_planner',
        executable='visual_track_node',
        name='visual_track_node',
        output='screen'
    )

    # 根据参数控制是否启动 mipi_detect
    conditional_mipi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mipi_detect'),
                'launch',
                'detect.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration("use_mipi_detect"))
    )

    return LaunchDescription([
        use_mipi_detect_arg,   # 把参数注册进来
        conditional_mipi,      # 受参数控制
        mission_planner,
        visual_track_node,
    ])
