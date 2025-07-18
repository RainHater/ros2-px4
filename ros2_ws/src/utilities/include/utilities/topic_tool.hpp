#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <mutex>

//话题消息监听可直接获取话题的消息
template<typename MsgT>
class TopicListener{
public:
    using MsgPtr = typename MsgT::SharedPtr;
    using SubscriptionPtr = typename rclcpp::Subscription<MsgT>::SharedPtr;
public:
    void subscribe(
        rclcpp::Node::SharedPtr node,
        const std::string & topic_name,
        const rclcpp::QoS &qos)
    {
        m_sub = node->create_subscription<MsgT>(
            topic_name, qos, 
            [this](const MsgPtr msg){
                std::lock_guard<std::mutex> lock(m_mutex);
                m_msg = *msg;
            }
        );
    }
    const MsgT& get_msg(){
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_msg;
    }

private:
    std::mutex m_mutex;
    SubscriptionPtr m_sub;
    MsgT m_msg;
};
