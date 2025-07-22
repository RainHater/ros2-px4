#ifndef _SET_OFFBOARD_MODE_API_H
#define _SET_OFFBOARD_MODE_API_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/set_offboard_mode.hpp"
#include "utilities/topic_name.hpp"

class SetOffboardModeApi{
public:
    using SetOffboardMode = common_msgs::action::SetOffboardMode;
public:
    SetOffboardModeApi(){
        m_is_busy = false;
    }
    static SetOffboardModeApi& Instance(){
        static SetOffboardModeApi api;

        return api; 
    }
    void send_goal(
        const rclcpp::Node::SharedPtr &node,
        const uint8_t arm_mode, 
        const uint8_t offboard_mode,
        std::function<void()> succeeded_callback)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_is_busy)
            return;

        m_client = rclcpp_action::create_client<SetOffboardMode>(
            node->shared_from_this(), topic_cli::SET_OFFBOARD_MODE
        );

        m_is_busy = true;
        if (!m_client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node->get_logger(), "Action server not available");
            m_is_busy = false;
            return;
        }

        SetOffboardMode::Goal goal;
        goal.mode.arming_state = arm_mode;
        goal.mode.offboard_mode = offboard_mode;

        RCLCPP_INFO(node->get_logger(), 
            "请求数据: arm: %d, offbaord: %d", 
                goal.mode.arming_state,
                goal.mode.offboard_mode);

        rclcpp_action::Client<SetOffboardMode>::SendGoalOptions options;
        options.goal_response_callback = [node](auto goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
            } else {
                RCLCPP_INFO(node->get_logger(), "Goal accepted by server");
            }
        };
        options.feedback_callback = [](auto, auto) {};
        options.result_callback = [this, succeeded_callback, node](
            const rclcpp_action::ClientGoalHandle<SetOffboardMode>::WrappedResult &result){
            {   
                std::lock_guard<std::mutex> lock(m_mutex);
                m_is_busy = false;
                m_client.reset();
            }
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    if (succeeded_callback)
                        succeeded_callback();
                    RCLCPP_INFO(node->get_logger(), "任务成功: %s",
                        result.result->message.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node->get_logger(), "任务被中止");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(node->get_logger(), "任务被取消");
                    break;
                default:
                    RCLCPP_ERROR(node->get_logger(), "未知任务状态");
                    break;
            }
        };
        m_client->async_send_goal(goal, options);
    }
private:
    bool m_is_busy;
    std::mutex m_mutex;
    rclcpp_action::Client<SetOffboardMode>::SharedPtr m_client;
};

#endif
