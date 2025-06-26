#ifndef _NAVIGATION_CONTROLLER_H
#define _NAVIGATION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/navigate_to_gps.hpp"

class NavigationController {
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToGPS>;

    NavigationController(const rclcpp::Node::SharedPtr &node);
    
    // 控制层对外暴露的接口
    void fly_to(double lat, double lon, double alt);
    bool get_task_status();
private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp_action::Client<NavigateToGPS>::SharedPtr m_nav_client;
    rclcpp_action::ResultCode m_result_code;

    void nav_goal_response_callback(std::shared_ptr<GoalHandleNavigate> future);
    void nav_feedback_callback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToGPS::Feedback> feedback);
    void nav_result_callback(const GoalHandleNavigate::WrappedResult &result);
};

#endif
