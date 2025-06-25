#include "flight_control/offboard_ctrl_node.h"
#include "utilities/util_topic.hpp"
#include <cmath>
#include <functional>

using std::placeholders::_1;

static constexpr float kInvalidThreshold = 1e-6f;

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting offboard_ctrl_node follower node...");
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10);
    
    m_target_setpoint_sub = create_subscription<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10, 
        std::bind(&OffboardCtrlNode::target_setpoint_callback, this, _1));
    m_current_offboard_mode_sub = utils::make_simple_subscription<
        common_msgs::msg::ArmOffboardStatus>(
      "/control/px4_mode_status_broadcaster",
        qos, this, px4_mode_status_broadcaster);

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&OffboardCtrlNode::timer_callback, this));
}

void OffboardCtrlNode::timer_callback(){
    publish_trajectory_setpoint();
}

void OffboardCtrlNode::target_setpoint_callback(const common_msgs::msg::TrajectorySetPoint::SharedPtr msg){
    auto &mode = px4_mode_status_broadcaster.offboard_mode;
    auto &POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
    auto &VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;

    if (mode == POSITION){
        utils::copy_float_data(msg->position, m_target_setpoint.position);
    }else if (mode == VELOCITY){
        utils::copy_float_data(msg->velocity, m_target_setpoint.velocity);
    }
    m_target_setpoint.yaw = msg->yaw;
    m_target_setpoint.yawspeed = msg->yawspeed;
}

void OffboardCtrlNode::publish_trajectory_setpoint() {
    const auto &target = m_target_setpoint;
    auto &msg = m_traj_msg_cache;
    auto &POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
    auto &VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
    auto &mode = px4_mode_status_broadcaster.offboard_mode;

    if (mode == POSITION && non_zero3(target.position)){
        utils::copy_float_data(target.position, msg.position);
        msg.yaw = target.yaw;
        msg.yawspeed = 0.0f;
    }else if (mode == VELOCITY && non_zero3(target.velocity)){
        utils::copy_float_data(target.velocity, msg.velocity);
        msg.yaw = NAN;
        msg.yawspeed = target.yawspeed;
    }
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectory_setpoint_pub->publish(msg);
}

bool OffboardCtrlNode::non_zero3(const std::array<float, 3>& v){
    return  std::fabs(v[0]) > kInvalidThreshold ||
            std::fabs(v[1]) > kInvalidThreshold ||
            std::fabs(v[2]) > kInvalidThreshold;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardCtrlNode>());

    rclcpp::shutdown();
    return 0;
}
