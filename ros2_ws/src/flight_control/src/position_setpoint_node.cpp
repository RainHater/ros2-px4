#include "flight_control/position_setpoint_node.h"
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using std::placeholders::_1;

PositionSetpointNode::PositionSetpointNode() 
    : Node("position_setpoint_node", "position_setpoint_node") {
    RCLCPP_INFO(get_logger(), "Starting position_setpoint_node follower node...");
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    m_target_position_sub = create_subscription<common_msgs::msg::PositionSetpoint>(
        "target_position", 10,
        std::bind(&PositionSetpointNode::target_position_callback, this, _1));
    m_current_position_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&PositionSetpointNode::current_position_callback, this, _1));
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&PositionSetpointNode::timer_callback, this));
}

void PositionSetpointNode::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    // 如果目标位置可用，则更新目标位置为 trajectory setpoint
    if (m_target_position.x != 0.0 || m_target_position.y != 0.0 || m_target_position.z != 0.0) {
        msg.position[0] = m_target_position.x;
        msg.position[1] = m_target_position.y;
        msg.position[2] = m_target_position.z;

        float dx = m_target_position.x - m_current_position.x;
        float dy = m_target_position.y - m_current_position.y;
        double dist = std::hypot(dx, dy);

        RCLCPP_DEBUG(get_logger(), "position dist: %f", dist);
        if (dist < 0.15){
            msg.yaw = m_last_yaw;
        }else {
            msg.yaw = std::atan2(dy, dx);
            m_last_yaw = msg.yaw;
        }
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    }
    m_trajectory_setpoint_pub->publish(msg);
}

void PositionSetpointNode::timer_callback(){

    publish_trajectory_setpoint();
}

void PositionSetpointNode::target_position_callback(const common_msgs::msg::PositionSetpoint msg) {
    m_target_position = msg;
    RCLCPP_INFO(this->get_logger(), "Received target position: [x: %.2f, y: %.2f, z: %.2f]",
                m_target_position.x, m_target_position.y, m_target_position.z);
}

void PositionSetpointNode::current_position_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
    m_current_position.x = msg->position[0];
    m_current_position.y = msg->position[1];
    m_current_position.z = msg->position[2];
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionSetpointNode>());

    rclcpp::shutdown();
    return 0;
}
