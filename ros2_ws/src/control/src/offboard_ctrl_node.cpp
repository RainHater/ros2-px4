#include "control/offboard_ctrl_node.h"
#include "utilities/util_topic.hpp"
#include <cmath>
#include <functional>
#include <sstream>
#include <iomanip>

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting offboard_ctrl_node follower node...");
    
    init_publisher();
    init_subscription();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&OffboardCtrlNode::timer_callback, this));
}

void OffboardCtrlNode::init_publisher(){
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10);
}

void OffboardCtrlNode::init_subscription(){
    m_target_setpoint_sub = utils::make_simple_subscription<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10, 
        this, 
        m_target_setpoint);

    m_current_offboard_mode_sub = utils::make_simple_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10, 
        this, 
        m_current_offboard_mode);
}

void OffboardCtrlNode::timer_callback(){
    publish_trajectory_setpoint();
}


void OffboardCtrlNode::publish_trajectory_setpoint() {
    const auto &target = m_target_setpoint;
    const auto &arm_state = m_current_offboard_mode.arming_state;
    const auto &mode = m_current_offboard_mode.offboard_mode;
    const auto &armed = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
    const auto &POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
    const auto &VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
    if (arm_state != armed)
        return;

    px4_msgs::msg::TrajectorySetpoint msg{};   
    auto use_if_mode = [&](auto& target_mode, float value) {
        return (mode == target_mode) ? value : NAN;
    };
    
    msg.position[0] = use_if_mode(POSITION, target.position[0]);
    msg.position[1] = use_if_mode(POSITION, target.position[1]);
    msg.position[2] = use_if_mode(POSITION, target.position[2]);
    msg.velocity[0] = use_if_mode(VELOCITY, target.velocity[0]);
    msg.velocity[1] = use_if_mode(VELOCITY, target.velocity[1]);
    msg.velocity[2] = use_if_mode(VELOCITY, target.velocity[2]);
    msg.yaw = use_if_mode(POSITION, target.yaw);
    msg.yawspeed = use_if_mode(VELOCITY, target.yawspeed);
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectory_setpoint_pub->publish(msg); 
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardCtrlNode>());

    rclcpp::shutdown();
    return 0;
}
