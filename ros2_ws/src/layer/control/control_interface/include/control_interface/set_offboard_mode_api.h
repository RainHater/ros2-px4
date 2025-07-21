#ifndef _SET_OFFBOARD_MODE_API_H
#define _SET_OFFBOARD_MODE_API_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/set_offboard_mode.hpp"

class SetOffboardModeApi{
public:
    using SetOffboardMode = common_msgs::action::SetOffboardMode;
public:
    SetOffboardModeApi();
    static SetOffboardModeApi& Instance();
    void send_goal(
        const rclcpp::Node::SharedPtr &node,
        const uint8_t arm_mode, 
        const uint8_t offboard_mode,
        std::function<void()> succeeded_callback
    );
private:
    bool m_is_busy;
    std::mutex m_mutex;
    rclcpp_action::Client<SetOffboardMode>::SharedPtr m_client;
};

#endif
