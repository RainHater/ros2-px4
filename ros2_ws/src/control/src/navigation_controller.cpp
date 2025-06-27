#include "control/navigation_controller.h"

using std::placeholders::_1;
using std::placeholders::_2;

NavigationController::NavigationController(
    const std::shared_ptr<rclcpp::Node> &node, 
    ResultCallback cb)
    : m_node(node), m_user_cb(cb) {
    m_client = rclcpp_action::create_client<NavigateToGPS>(
        m_node, "/control/navigate_to_gps");
}

void NavigationController::send_goal(double lat, double lon, double alt){
    if (m_is_busy)
        return;

    m_is_busy = true;
    if (!m_client->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(m_node->get_logger(), "Action server not available");
        return;
    }
    NavigateToGPS::Goal goal;
    goal.lat = lat;
    goal.lon = lon;
    goal.alt = alt;

    RCLCPP_INFO(m_node->get_logger(), "Sending goal: lat=%f lon=%f alt=%f", lat, lon, alt);

    rclcpp_action::Client<NavigateToGPS>::SendGoalOptions options;
    options.goal_response_callback = [this](auto goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected");
        } else {
            RCLCPP_INFO(m_node->get_logger(), "Goal accepted by server");
        }
    };
    options.feedback_callback = [](auto, auto) {};
    options.result_callback = std::bind(&NavigationController::handle_result, this, _1);
    m_client->async_send_goal(goal, options);
}

void NavigationController::handle_result(const GoalHandle::WrappedResult &result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(m_node->get_logger(), "导航成功: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(m_node->get_logger(), "导航任务被中止");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(m_node->get_logger(), "导航任务被取消");
            break;
        default:
            RCLCPP_ERROR(m_node->get_logger(), "未知导航结果状态");
            break;
    }

    if (m_user_cb)
        m_user_cb(result);
    m_is_busy = false;
}
