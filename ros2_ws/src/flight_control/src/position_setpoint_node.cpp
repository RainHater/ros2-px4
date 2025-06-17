#include "flight_control/position_setpoint_node.h"

PositionSetpointNode::PositionSetpointNode() : Node("position_setpoint_node") {

    m_trajectory_setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    m_goal_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PositionSetpointNode::goal_pose_callback, this, std::placeholders::_1));

    auto timer_callback = [this]() -> void {
        publish_trajectory_setpoint();
    };

    m_timer = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

void PositionSetpointNode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    m_goal_pose = *msg;
    // RCLCPP_INFO(this->get_logger(), "take over pose information!");
    // RCLCPP_INFO(this->get_logger(), "Received goal pose: [x: %.2f, y: %.2f, z: %.2f]",
    //             m_goal_pose.pose.position.x, m_goal_pose.pose.position.y, m_goal_pose.pose.position.z);
}

void PositionSetpointNode::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    // 如果目标位置可用，则更新目标位置为 trajectory setpoint
    if (m_goal_pose.pose.position.x != 0.0 || m_goal_pose.pose.position.y != 0.0 || m_goal_pose.pose.position.z != 0.0) {
        msg.position[0] = m_goal_pose.pose.position.x;
        msg.position[1] = m_goal_pose.pose.position.y;
        msg.position[2] = m_goal_pose.pose.position.z;
        msg.yaw = 0.0;  // 可以根据需要设置目标航向
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    }

    m_trajectory_setpoint_publisher->publish(msg);
}

int main(int argc, char *argv[]) {
    std::cout << "Starting position_setpoint_node follower node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionSetpointNode>());

    rclcpp::shutdown();
    return 0;
}
