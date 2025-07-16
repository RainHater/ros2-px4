#pragma once

#include <common_msgs/msg/detail/trajectory_set_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace topic_sub_tool {
template<typename MsgT>
static void create_subscription(
    const rclcpp::Node::SharedPtr & node,
    typename  rclcpp::Subscription<MsgT>::SharedPtr & sub,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(const typename MsgT::SharedPtr)> callback)
{
    sub = node->create_subscription<MsgT>(
        topic_name,
        qos,
        callback
    );
}
inline void control_trajectory_setpoint(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Subscription<common_msgs::msg::TrajectorySetPoint>::SharedPtr &sub,
    std::function<void(const common_msgs::msg::TrajectorySetPoint::SharedPtr)> callback)
{   
    create_subscription<common_msgs::msg::TrajectorySetPoint>(
        node, sub, "/control/trajectory_setpoint", 10, callback);
}

inline void control_px4_mode_status(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr &sub,
    std::function<void(const common_msgs::msg::ArmOffboardStatus::SharedPtr)> callback)
{   
    create_subscription<common_msgs::msg::ArmOffboardStatus>(
        node, sub, "/control/px4_mode_status_broadcaster", 10, callback);
}

inline void control_set_offboard_mode(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr &sub,
    std::function<void(const common_msgs::msg::ArmOffboardStatus::SharedPtr)> callback)
{   
    create_subscription<common_msgs::msg::ArmOffboardStatus>(
        node, sub, "/control/set_offboard_mode", 10, callback);
}

inline void vehicle_status(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr &sub,
    std::function<void(const px4_msgs::msg::VehicleStatus::SharedPtr)> callback)
{   
    create_subscription<px4_msgs::msg::VehicleStatus>(
        node, sub, "/interface/out/vehicle_status", 10, callback);
}

inline void vehicle_local_position(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr &sub,
    std::function<void(const px4_msgs::msg::VehicleLocalPosition::SharedPtr)> callback)
{   
    create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        node, sub, "/interface/out/vehicle_local_position", 10, callback);
}

inline void vehicle_odometry(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr &sub,
    std::function<void(const px4_msgs::msg::VehicleOdometry::SharedPtr)> callback)
{   
    create_subscription<px4_msgs::msg::VehicleOdometry>(
        node, sub, "/interface/out/vehicle_odometry", 10, callback);
}
}
