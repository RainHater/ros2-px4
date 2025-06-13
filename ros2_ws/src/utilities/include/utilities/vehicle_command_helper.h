#pragma once

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

namespace flight_control {
class VehicleCommandHelper
{
public:
    VehicleCommandHelper(rclcpp::Node *node);
    void publish(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
private:
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_publisher;
    rclcpp::Node *m_node;
};
}
