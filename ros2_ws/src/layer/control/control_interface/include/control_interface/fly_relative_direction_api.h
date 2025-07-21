#ifndef _FLY_RELATIVE_DIRECTION_H
#define _FLY_RELATIVE_DIRECTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/fly_relative_direction.hpp"

#include <mutex>

class FlyRelativeDirectionApi{
public:
    using FlyRelative = common_msgs::action::FlyRelativeDirection;
public:
    FlyRelativeDirectionApi();
    static FlyRelativeDirectionApi& Instance();
    void send_goal(const rclcpp::Node::SharedPtr &node,
        float forward, float right, float up,
        std::function<void()> succeeded_callback
    );
private:
    bool m_is_busy;
    std::mutex m_mutex;
    rclcpp_action::Client<FlyRelative>::SharedPtr m_client;
};


#endif
