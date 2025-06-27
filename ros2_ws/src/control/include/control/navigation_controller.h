#ifndef _NAVIGATION_CONTROLLER_H
#define _NAVIGATION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/navigate_to_gps.hpp"

class NavigationController{
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToGPS>;
    using ResultCallback = std::function<void(const GoalHandle::WrappedResult&)>;
public:
    NavigationController(
        const std::shared_ptr<rclcpp::Node> &node, 
        ResultCallback cb);
    void send_goal(double lat, double lon, double alt);
private: 
    void handle_result(const GoalHandle::WrappedResult &result);
private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp_action::Client<NavigateToGPS>::SharedPtr m_client;
    ResultCallback m_user_cb;
    bool m_is_busy;
};

#endif
