#include "control/controller_api.h"

namespace ControllerApi{
NavApi::NavApi(){
    m_nav_is_busy = false;
}

NavApi& NavApi::Instance(){
    static NavApi api;
    
    return api;
}

void NavApi::send_goal(const rclcpp::Node::SharedPtr &node, 
    rclcpp_action::Client<NavigateToGPS>::SharedPtr &nav_clinet, 
    double lat, double lon, double alt, std::function<void()> succeeded_callback){
    if (m_nav_is_busy)
        return;

    m_nav_is_busy = true;
    if (!nav_clinet->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available");
        m_nav_is_busy = false;
        return;
    }
    NavigateToGPS::Goal goal;
    goal.lat = lat;
    goal.lon = lon;
    goal.alt = alt;

    RCLCPP_INFO(node->get_logger(), "Sending goal: lat=%f lon=%f alt=%f", lat, lon, alt);

    rclcpp_action::Client<NavigateToGPS>::SendGoalOptions options;
    options.goal_response_callback = [node](auto goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
        } else {
            RCLCPP_INFO(node->get_logger(), "Goal accepted by server");
        }
    };
    options.feedback_callback = [](auto, auto) {};
    options.result_callback = [this, succeeded_callback, node](const GoalHandle::WrappedResult &result){
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (succeeded_callback)
                succeeded_callback();
            RCLCPP_INFO(node->get_logger(), "导航成功: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "导航任务被中止");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node->get_logger(), "导航任务被取消");
            break;
        default:
            RCLCPP_ERROR(node->get_logger(), "未知导航结果状态");
            break;
        }
        m_nav_is_busy = false;
    };
    nav_clinet->async_send_goal(goal, options);
}
}
