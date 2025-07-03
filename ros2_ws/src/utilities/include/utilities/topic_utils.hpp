#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace topic_utils {

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

template <typename ContainerSrc, typename ContainerDst>
void copy_float_data(const ContainerSrc& src, ContainerDst& dst) {
    static_assert(std::is_same<typename ContainerSrc::value_type, float>::value, "src must hold float");
    static_assert(std::is_same<typename ContainerDst::value_type, float>::value, "dst must hold float");
    std::copy(std::begin(src), std::end(src), std::begin(dst));
}

}
