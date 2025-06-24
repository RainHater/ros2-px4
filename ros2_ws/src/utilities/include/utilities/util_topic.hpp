#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace utils {

template<typename MsgT, typename MemberT>
inline typename rclcpp::Subscription<MsgT>::SharedPtr
make_simple_subscription(
    const std::string &topic,
    const rclcpp::QoS &qos,
    rclcpp::Node *node,
    MemberT &member)
{
    return node->create_subscription<MsgT>(
        topic, qos,
        [&member](const typename MsgT::SharedPtr msg) {
            member = *msg;
        }
    );
}

}
