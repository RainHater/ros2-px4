#ifndef _MISSION_PLANNER_NODE_H
#define _MISSION_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/target_gps.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>

enum TaskStatus {
    WAIT_FOR_ARM_AND_OFFBOARD = 0,
    FLY_TO_READY_POSITION,
    FLY_TO_GPS_TARGET,
};

class MissionPlanner : public rclcpp::Node {
public:
    MissionPlanner();
protected:
    //定时器回调函数
    void timer_callback();
    void px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus &msg);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布GPS位置
    rclcpp::Publisher<common_msgs::msg::TargetGps>::SharedPtr m_trajectory_setpoint_pub;
    //订阅当前飞控arm和offboard状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_sub;

    TaskStatus m_current_task_status = WAIT_FOR_ARM_AND_OFFBOARD;
};

#endif
