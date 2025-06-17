#ifndef _GPS_NAVIGATION_NODE_H
#define _GPS_NAVIGATION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <common_msgs/msg/target_gps.hpp>

class GpsNavigationNode : public rclcpp::Node {
public:
    GpsNavigationNode();
protected:
    void global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void target_gps_callback(const common_msgs::msg::TargetGps::SharedPtr msg);
private:
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_global_position_subscription;
    rclcpp::Subscription<common_msgs::msg::TargetGps>::SharedPtr m_target_gps;
};

#endif
