#include "flight_control/mission_planner_node.h"
#include <common_msgs/msg/detail/target_gps__struct.hpp>

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner_node") {
    RCLCPP_INFO(get_logger(), "Starting mission_planner_node follower node...");
    
    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TargetGps>(
        "/control/target_gps", 10);

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MissionPlanner::timer_callback, this));
}

void MissionPlanner::timer_callback(){

}

