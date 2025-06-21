#include "flight_control/velocity_ctrl_node.h"

VelocityCtrlNode::VelocityCtrlNode()
    : Node("velocity_ctrl_node", "velocity_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting velocity_ctrl_node follower node...");


}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityCtrlNode>());

    rclcpp::shutdown();
    return 0;
}