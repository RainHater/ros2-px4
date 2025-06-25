#ifndef _MISSION_PLANNER_NODE_H
#define _MISSION_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/target_gps.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/action/navigate_to_gps.hpp>

using NavigateToGPS = common_msgs::action::NavigateToGPS;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToGPS>;

enum TaskStatus {
    WAIT_FOR_ARM_AND_OFFBOARD = 0,
    FLY_TO_READY_POSITION,
    FLY_TO_GPS_TARGET,
};

class MissionPlanner : public rclcpp::Node {
public:
    MissionPlanner();
protected:
    void timer_callback();
    //px4模式获取
    void px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg);
    void nav_goal_response_callback(std::shared_ptr<GoalHandleNavigate> future);
    void nav_feedback_callback(
        GoalHandleNavigate::SharedPtr, 
        const std::shared_ptr<const NavigateToGPS::Feedback> feedback);
    void nav_result_callback(const GoalHandleNavigate::WrappedResult &result);
    void nav_sends_goal();
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //经纬度导航客户端
    rclcpp_action::Client<NavigateToGPS>::SharedPtr m_nav_cllient;
    //发布GPS位置
    rclcpp::Publisher<common_msgs::msg::TargetGps>::SharedPtr m_trajectory_setpoint_pub;
    //订阅当前飞控arm和offboard状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_sub;
    TaskStatus m_current_task_status = WAIT_FOR_ARM_AND_OFFBOARD;
    bool m_action_state = false;
};

#endif
