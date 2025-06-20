#include "flight_control/velocity_ctrl_node.h"

VelocityCtrlNode::VelocityCtrlNode()
    : Node("velocity_ctrl_node", "velocity_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting velocity_ctrl_node follower node...");


}

