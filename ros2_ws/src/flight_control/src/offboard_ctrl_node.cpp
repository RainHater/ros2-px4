#include "flight_control/offboard_ctrl_node.h"
#include "utilities/util_topic.hpp"
#include <cmath>

static constexpr float kDistThreshold    = 0.15f;
static constexpr float kInvalidThreshold = 1e-6f;

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting offboard_ctrl_node follower node...");
    
    m_task_status.status = common_msgs::msg::TaskStatus::TASK_UNKNOWN;
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10);
    
    m_current_setpoint_sub = utils::make_simple_subscription<
        px4_msgs::msg::VehicleOdometry>(
        "/interface/out/vehicle_odometry", 
        qos, this, m_current_setpoint);

    m_trajectory_setpoint_sub = utils::make_simple_subscription<
        px4_msgs::msg::VehicleOdometry>(
      "/control/trajectory_setpoint", 
        qos, this, m_target_setpoint);
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

void OffboardCtrlNode::publish_trajectory_setpoint() {
    const auto &target = m_target_setpoint;
    auto &msg = m_traj_msg_cache;
    auto &mode = px4_mode_status_broadcaster.offboard_mode;
    // const auto &current = m_current_setpoint;

    if (mode == common_msgs::msg::ArmOffboardStatus::POSITION
        && non_zero3(target.position)){
        std::copy(std::begin(target.position),
            std::end(target.position),
            std::begin(msg.position));
    }else if (mode == common_msgs::msg::ArmOffboardStatus::VELOCITY
        && non_zero3(target.velocity)){
        std::copy(std::begin(target.velocity),
            std::end(target.velocity),
            std::begin(msg.velocity));
    }

    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectory_setpoint_pub->publish(msg);
}

bool OffboardCtrlNode::non_zero3(const std::array<float, 3>& v){
    return  std::fabs(v[0]) > kInvalidThreshold ||
            std::fabs(v[1]) > kInvalidThreshold ||
            std::fabs(v[2]) > kInvalidThreshold;
}

void OffboardCtrlNode::fill_position(
    const px4_msgs::msg::VehicleOdometry &target, 
    px4_msgs::msg::TrajectorySetpoint &msg){
    if (non_zero3(target.position)) {
        std::copy(std::begin(target.position),
                std::end  (target.position),
                std::begin(msg.position));
    }
    msg.yaw = NAN;
    msg.yawspeed = 0.0f;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardCtrlNode>());

    rclcpp::shutdown();
    return 0;
}

// void OffboardCtrlNode::offboard_position_mode(px4_msgs::msg::TrajectorySetpoint &msg){

//     const auto &target = m_target_setpoint;
//     const auto &current = m_current_setpoint;

//     if (std::abs(target.position[0]) < INVALID_THRESHOLD &&
//         std::abs(target.position[1]) < INVALID_THRESHOLD &&
//         std::abs(target.position[2]) < INVALID_THRESHOLD) {
//         return;
//     }

//     msg.position[0] = target.position[0];
//     msg.position[1] = target.position[1];
//     msg.position[2] = target.position[2];

//     float dx = target.position[0] - current.position[0];
//     float dy = target.position[1] - current.position[1];
//     float dz = target.position[2] - current.position[2];

//     if (!is_at_target_position(dx, dy, dz)){
//         m_last_yaw = std::atan2(dy, dx);
//     }
//     msg.yaw = m_last_yaw;  
// }

// void OffboardCtrlNode::offboard_velocity_mode(px4_msgs::msg::TrajectorySetpoint &msg){
//     const float EPS_VEL = 1e-3f;
    
//     const auto &setpoint = m_target_setpoint;
//     // const auto &current = m_current_setpoint;

//     if (std::fabs(setpoint.velocity[0]) < EPS_VEL &&
//         std::fabs(setpoint.velocity[1]) < EPS_VEL &&
//         std::fabs(setpoint.velocity[2]) < EPS_VEL) {
//         return;
//     }

//     msg.velocity[0] = setpoint.velocity[0];
//     msg.velocity[1] = setpoint.velocity[1];
//     msg.velocity[2] = setpoint.velocity[2];
// }

// bool OffboardCtrlNode::is_at_target_position(float dx, float dy, float dz) {

//     float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
//     if (dist < DIST_THRESHOLD) {
//         m_task_status.status = common_msgs::msg::TaskStatus::TASK_COMPLETED;
//         return true;
//     }
//     return false;
// }

