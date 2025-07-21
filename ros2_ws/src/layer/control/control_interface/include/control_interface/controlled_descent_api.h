#ifndef _CONTROLLED_DESCENT_H
#define _CONTROLLED_DESCENT_H

#include <common_msgs/action/detail/controlled_descent__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/controlled_descent.hpp"

#include <mutex>

class ControlledDescentApi{
public:
    using ControlledDescent = common_msgs::action::ControlledDescent;
public:
    ControlledDescentApi();
    static ControlledDescentApi& Instance();
    void send_goal(const rclcpp::Node::SharedPtr &node,
        float speed,
        std::function<void()> succeeded_callback
    );
private:
    bool m_is_busy;
    std::mutex m_mutex;
    rclcpp_action::Client<ControlledDescent>::SharedPtr m_client;
};

#endif
