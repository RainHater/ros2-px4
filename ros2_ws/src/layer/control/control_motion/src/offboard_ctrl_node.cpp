#include "control_motion/offboard_ctrl_node.h"
#include "utilities/topic_name.hpp"
#include <cmath>
#include <functional>
#include <sstream>
#include <iomanip>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") 
{
    RCLCPP_INFO(get_logger(), "offboard_ctrl_node 节点启动...");
}

void OffboardCtrlNode::initialize(){
    init_publisher();
    init_subscription();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&OffboardCtrlNode::timer_callback, this));
}

void OffboardCtrlNode::init_publisher(){
    m_pub.trajectory_setpoint = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_pub::PX4_TRAJECTORY_SETPOINT, 10);
}

void OffboardCtrlNode::init_subscription(){
    m_listener.target_setpoint.subscribe(
        shared_from_this(), 
        topic_sub::TRAJECTORY_SETPOINT, 10
    );
    m_listener.current_offboard_mode.subscribe(
        shared_from_this(), 
        topic_sub::PX4_MODE, 10
    );
}

void OffboardCtrlNode::timer_callback(){
    publish_trajectory_setpoint();
}

void OffboardCtrlNode::publish_trajectory_setpoint() {
    auto &target_setpoint= m_listener.target_setpoint.get_msg();
    auto &current_offboard_mode= m_listener.current_offboard_mode.get_msg();

    if (current_offboard_mode.arm == ARM_DISABLED)
        return;
    auto msg = target_setpoint;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_pub.trajectory_setpoint->publish(msg);
}

// int main(int argc, char *argv[]) {
//     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<OffboardCtrlNode>();
//     node->initialize();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
