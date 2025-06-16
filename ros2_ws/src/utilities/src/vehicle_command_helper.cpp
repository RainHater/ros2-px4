#include "utilities/vehicle_command_helper.h"

namespace utilities {
VehicleCommandHelper::VehicleCommandHelper(rclcpp::Node *node)
: m_node(node) {
    m_publisher = m_node->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
}

void VehicleCommandHelper::publish(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = m_node->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    m_publisher->publish(msg);
}

} 
