#include "mission_planner/px4_hold_height_test_node.h"

Px4HoldHeightTestNode::Px4HoldHeightTestNode()
    : rclcpp::Node("px4_hold_height_test_node") 
{
    
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4HoldHeightTestNode>());
    rclcpp::shutdown();
    return 0;
}
