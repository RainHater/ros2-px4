from launch import LaunchDescription
from launch_ros.actions import Node

#发布位置
# ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 5.0, y: 20.0, z: -10.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_control_follower',
            executable='trajectory_follower_node',
            output='screen'
        )
    ])
