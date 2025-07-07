#include "mission_planner/px4_hold_height_test_node.h"

Px4HoldHeightTestNode::Px4HoldHeightTestNode()
    : rclcpp::Node("px4_hold_height_test_node") 
{
    RCLCPP_INFO(get_logger(), "Starting px4_hold_height_test_node follower node...");

    init_publisher();
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&Px4HoldHeightTestNode::timer_callback, this));
}

void Px4HoldHeightTestNode::init_publisher(){
    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
}

void Px4HoldHeightTestNode::timer_callback(){
    
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4HoldHeightTestNode>());
    rclcpp::shutdown();
    return 0;
}
