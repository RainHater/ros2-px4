#ifndef _MISSION_PLANNER_NODE_H
#define _MISSION_PLANNER_NODE_H

#include <common_msgs/msg/detail/target_gps__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/control_mode.hpp>
#include <common_msgs/msg/target_gps.hpp>

enum TaskStatus {
    
};

class MissionPlanner : public rclcpp::Node {
public:
    MissionPlanner();
protected:
    //定时器回调函数
    void timer_callback();
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布GPS位置
    rclcpp::Publisher<common_msgs::msg::TargetGps>::SharedPtr m_trajectory_setpoint_pub;
};

#endif
