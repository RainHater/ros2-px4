#include "control/navigation_controller.h"

using std::placeholders::_1;
using std::placeholders::_2;

NavigationController::NavigationController(const std::shared_ptr<rclcpp::Node> &node)
    : m_node(node) {
    m_nav_client = rclcpp_action::create_client<NavigateToGPS>(
        m_node, "/control/navigate_to_gps");
}

void NavigationController::fly_to(double lat, double lon, double alt, ResultCallback cb) {
    if (m_is_busy)
        return;

    if (!m_nav_client->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(m_node->get_logger(), "NavigateToGPS action server not available");
        return;
    }

    NavigateToGPS::Goal goal;
    goal.lat = lat;
    goal.lon = lon;
    goal.alt = alt;

    RCLCPP_INFO(m_node->get_logger(), "Sending goal: lat=%f lon=%f alt=%f", lat, lon, alt);

    rclcpp_action::Client<NavigateToGPS>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
        &NavigationController::nav_goal_response_callback, this, _1);
    options.feedback_callback = std::bind(
        &NavigationController::nav_feedback_callback, this, _1, _2);
    options.result_callback = std::bind(
        &NavigationController::nav_result_callback, this, _1);;
    
    m_user_result_cb = cb;
    m_nav_client->async_send_goal(goal, options);
    m_is_busy = true;
}

void NavigationController::nav_goal_response_callback(std::shared_ptr<GoalHandleNavigate> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(m_node->get_logger(), "Goal accepted by server");
    }
}

void NavigationController::nav_feedback_callback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const NavigateToGPS::Feedback> feedback) {
    (void)feedback;
    // RCLCPP_INFO(m_node->get_logger(), "Feedback: lat=%f lon=%f alt=%f remain=%.2f",
    //             feedback->current_latitude,
    //             feedback->current_longitude,
    //             feedback->current_altitude,
    //             feedback->distance_remaining);
}

void NavigationController::nav_result_callback(const GoalHandleNavigate::WrappedResult &result){
    if (m_user_result_cb)
        m_user_result_cb(result);

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(m_node->get_logger(), "Result: success=%d message=%s", result.result->success, result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(m_node->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(m_node->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(m_node->get_logger(), "Unknown result code");
            break;
    }
    m_is_busy = false;
}
