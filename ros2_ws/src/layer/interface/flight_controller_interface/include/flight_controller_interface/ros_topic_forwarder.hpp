#pragma once

#include <rclcpp/rclcpp.hpp>

template<typename MsgT>
class TopicForwarder{
public:
    using PublisherPtr = typename rclcpp::Publisher<MsgT>::SharedPtr;
    using SubscriptionPtr = typename rclcpp::Subscription<MsgT>::SharedPtr;
public:
    void create_forwarding_subscription(
        rclcpp::Node::SharedPtr node,
        const std::string & input_topic,
        const std::string & output_topic,
        const rclcpp::QoS &qos)
    {
        m_pub = node->create_publisher<MsgT>(output_topic, 10);
        m_sub = node->create_subscription<MsgT>(
        input_topic, qos,
        [this](const typename MsgT::SharedPtr msg){
            m_pub->publish(*msg);
        }
    );
    }
private:
    PublisherPtr m_pub;
    SubscriptionPtr m_sub;
};

