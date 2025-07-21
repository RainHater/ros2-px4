#include "control_interface/controlled_descent_api.h"
#include "utilities/topic_name.hpp"

ControlledDescentApi::ControlledDescentApi(){
    m_is_busy = false;
}

ControlledDescentApi& ControlledDescentApi::Instance(){
    static ControlledDescentApi api;

    return api;
}

void ControlledDescentApi::send_goal(const rclcpp::Node::SharedPtr &node,
    float speed,
    std::function<void()> succeeded_callback)
{   
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_is_busy)
        return;

    m_client = rclcpp_action::create_client<ControlledDescent>(
        node->shared_from_this(), topic_cli::CONTROLLED_DESCENT
    );

    m_is_busy = true;
    if (!m_client->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available");
        m_is_busy = false;
        return;
    }

    ControlledDescent::Goal goal;
    goal.speed = speed;

    RCLCPP_INFO(node->get_logger(), 
        "Sending goal: speed=%f", 
        speed);
    
    rclcpp_action::Client<ControlledDescent>::SendGoalOptions options;
    options.goal_response_callback = [node](auto goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
        } else {
            RCLCPP_INFO(node->get_logger(), "Goal accepted by server");
        }
    };
    options.feedback_callback = [](auto, auto) {};
    options.result_callback = [this, succeeded_callback, node](
        const rclcpp_action::ClientGoalHandle<ControlledDescent>::WrappedResult &result){
        {   
            std::lock_guard<std::mutex> lock(m_mutex);
            m_is_busy = false;
            m_client.reset();
        }
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
    };
    m_client->async_send_goal(goal, options);
}
