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
    switch(m_current_task_status){
        case FLY_TO_GPS_TARGET:{
            common_msgs::msg::TargetGps target_gps;
            target_gps.lat = 0.0000047;
            target_gps.lon = 0.0000009;
            target_gps.alt = 2.0;
            m_trajectory_setpoint_pub->publish(target_gps);
        }
        break;
    }
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanner>());

    rclcpp::shutdown();
    return 0;
}
