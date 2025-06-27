#ifndef _NAVIGATION_CONTROLLER_H
#define _NAVIGATION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/navigate_to_gps.hpp"

class NavigationController {
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToGPS>;
    using ResultCallback = std::function<void(GoalHandleNavigate::WrappedResult)>;
public:
    NavigationController(const std::shared_ptr<rclcpp::Node> &node);
    
    void fly_to(double lat, double lon, double alt, ResultCallback cb);
private:
    void nav_goal_response_callback(std::shared_ptr<GoalHandleNavigate> future);
    void nav_feedback_callback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToGPS::Feedback> feedback);
    void nav_result_callback(const GoalHandleNavigate::WrappedResult &result);
private:
    //ROS 2 Node 和 Action 客户端
    rclcpp::Node::SharedPtr m_node;
    rclcpp_action::Client<NavigateToGPS>::SharedPtr m_nav_client;
    
    //用户自定义的结果回调
    ResultCallback m_user_result_cb;
    //任务进行中
    bool m_is_busy = false;
};

#endif
