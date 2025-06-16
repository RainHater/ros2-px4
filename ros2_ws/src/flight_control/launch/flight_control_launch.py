from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_node = Node(
        package='flight_control',
        executable='arming_offboard_node',
        name='arming_offboard_node',
        output='screen',
    )

    # offboard_node = Node(
    #     package='flight_control',
    #     executable='offboard_control_node',
    #     name='offboard_control_node',
    #     output='screen',
    # )

    return LaunchDescription([
        arm_node,
        # offboard_node,
    ])
