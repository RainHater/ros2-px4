#include "flight_control/offboard_ctrl_node.h"
#include <cmath>
#include <limits>

using std::placeholders::_1;

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting offboard_ctrl_node follower node...");
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10);
    m_current_position_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/interface/out/vehicle_odometry", qos,
        std::bind(&OffboardCtrlNode::current_position_callback, this, _1));
    m_trajectory_setpoint_sub = create_subscription<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10,
        std::bind(&OffboardCtrlNode::trajectory_setpoint_callback, this, _1));
    m_current_offboard_mode_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", qos,
        std::bind(&OffboardCtrlNode::current_offboard_callback, this, _1));
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&OffboardCtrlNode::timer_callback, this));
}

void OffboardCtrlNode::timer_callback(){
    publish_trajectory_setpoint();
}

void OffboardCtrlNode::trajectory_setpoint_callback(const common_msgs::msg::TrajectorySetPoint msg) {
    m_trajectory_setpoint = msg;
    RCLCPP_INFO(this->get_logger(), "Received target position: [x: %.2f, y: %.2f, z: %.2f]",
                m_trajectory_setpoint.position[0], 
                m_trajectory_setpoint.position[1], 
                m_trajectory_setpoint.position[2]);
}

void OffboardCtrlNode::current_position_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
    m_current_setpoint.position[0] = msg->position[0];
    m_current_setpoint.position[1] = msg->position[1];
    m_current_setpoint.position[2] = msg->position[2];
}

void OffboardCtrlNode::current_offboard_callback(const common_msgs::msg::ArmOffboardStatus msg){
    px4_mode_status_broadcaster = msg;
}

void OffboardCtrlNode::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    offboard_position_mode(msg);
    offboard_velocity_mode(msg);

    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectory_setpoint_pub->publish(msg);
}

void OffboardCtrlNode::offboard_position_mode(px4_msgs::msg::TrajectorySetpoint &msg){
    const float DIST_THRESHOLD = 0.15f;

    const auto &setpoint = m_trajectory_setpoint;
    const auto &current = m_current_setpoint;

    if (px4_mode_status_broadcaster.offboard_mode != common_msgs::msg::ArmOffboardStatus::POSITION) {
        msg.position[0] = std::numeric_limits<float>::quiet_NaN();
        msg.position[1] = std::numeric_limits<float>::quiet_NaN();
        msg.position[2] = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if (setpoint.position[0] == 0.0f && setpoint.position[1] == 0.0f && setpoint.position[2] == 0.0f) {
        return;
    }

    msg.position[0] = setpoint.position[0];
    msg.position[1] = setpoint.position[1];
    msg.position[2] = setpoint.position[2];

    float dx = setpoint.position[0] - current.position[0];
    float dy = setpoint.position[1] - current.position[1];
    float dist = std::hypot(dx, dy);

    if (dist < DIST_THRESHOLD) {
        msg.yaw = m_last_yaw;
    } else {
        msg.yaw = std::atan2(dy, dx);
        m_last_yaw = msg.yaw;
    }
}

void OffboardCtrlNode::offboard_velocity_mode(px4_msgs::msg::TrajectorySetpoint &msg){
    const float EPS_VEL = 1e-3f;
    
    const auto &setpoint = m_trajectory_setpoint;
    // const auto &current = m_current_setpoint;

    if (px4_mode_status_broadcaster.offboard_mode != common_msgs::msg::ArmOffboardStatus::VELOCITY) {
        msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
        msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
        msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if (std::fabs(setpoint.velocity[0]) < EPS_VEL &&
        std::fabs(setpoint.velocity[1]) < EPS_VEL &&
        std::fabs(setpoint.velocity[2]) < EPS_VEL) {
        return;
    }

    msg.velocity[0] = setpoint.velocity[0];
    msg.velocity[1] = setpoint.velocity[1];
    msg.velocity[2] = setpoint.velocity[2];
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardCtrlNode>());

    rclcpp::shutdown();
    return 0;
}
