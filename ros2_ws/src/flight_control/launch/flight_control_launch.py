from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_offboard_node = Node(
        package='flight_control',
        executable='arming_offboard_node',
        name='arming_offboard_node',
        output='screen',
    )

    position_setpoint_node = Node(
        package='flight_control',
        executable='position_setpoint_node',
        name='position_setpoint_node',
        output='screen',
    )

    return LaunchDescription([
        arm_offboard_node,
        position_setpoint_node,
    ])
