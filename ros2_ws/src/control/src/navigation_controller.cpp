#include "control/navigation_controller.h"

NavigationController::NavigationController(const rclcpp::Node::SharedPtr &node)
    : m_node(node) {
    m_nav_client = rclcpp_action::create_client<NavigateToGPS>(
        m_node, "/control/navigate_to_gps");
}

void NavigationController::fly_to(double lat, double lon, double alt) {
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
        &NavigationController::nav_goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(
        &NavigationController::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(
        &NavigationController::nav_result_callback, this, std::placeholders::_1);

    m_nav_client->async_send_goal(goal, options);
}

bool NavigationController::get_task_status(){
    return (m_result_code == rclcpp_action::ResultCode::SUCCEEDED);
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
    RCLCPP_INFO(m_node->get_logger(), "Feedback: lat=%f lon=%f alt=%f remain=%.2f",
                feedback->current_latitude,
                feedback->current_longitude,
                feedback->current_altitude,
                feedback->distance_remaining);
}

void NavigationController::nav_result_callback(const GoalHandleNavigate::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(m_node->get_logger(), "Goal succeeded: %s", result.result->message.c_str());
    } else {
        RCLCPP_ERROR(m_node->get_logger(), "Goal failed with code %d", static_cast<int>(result.code));
    }

    m_result_code = result.code;
}
