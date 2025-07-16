#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>

namespace topic_pub_tool {
template<typename MsgT>
static void create_publisher(
    const rclcpp::Node::SharedPtr & node,
    typename  rclcpp::Publisher<MsgT>::SharedPtr & pub,
    const std::string & topic_name,
    const rclcpp::QoS & qos)
{
    pub = node->create_publisher<MsgT>(
        topic_name,
        qos
    );
}

inline void trajectory_setpoint(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr &pub)
{
    create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        node, pub, "/interface/in/trajectory_setpoint", 10);
}

inline void offboard_control_mode(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr &pub)
{
    create_publisher<px4_msgs::msg::OffboardControlMode>(
        node, pub, "/interface/in/offboard_control_mode", 10);
}

inline void vehicle_command(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr &pub)
{
    create_publisher<px4_msgs::msg::VehicleCommand>(
        node, pub, "/interface/in/vehicle_command", 10);
}

inline void control_px4_mode_status(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr &pub)
{
    create_publisher<common_msgs::msg::ArmOffboardStatus>(
        node, pub, "/control/px4_mode_status_broadcaster", 10);
}

inline void control_trajectory_setpoint(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr &pub)
{
    create_publisher<common_msgs::msg::TrajectorySetPoint>(
        node, pub, "/control/trajectory_setpoint", 10);
}

inline void control_set_offboard_mode(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr &pub)
{
    create_publisher<common_msgs::msg::ArmOffboardStatus>(
        node, pub, "/control/set_offboard_mode", 10);
}
}
