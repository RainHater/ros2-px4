#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

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
                m_msg = *msg;
                if (!m_first_flag){
                    m_first_msg = m_msg;
                    m_first_flag = true;
                }
                m_change_flag = true;
                m_received = true;
            }
        );
    }

    const MsgT& get_msg() const{
        return m_msg;
    }

    //获取第一次接收的数据
    const MsgT& get_first_msg() const{
        return m_first_msg;
    }

    //监听数据是否有效
    bool has_received() const {
        return m_received;
    }

    bool has_change() {
        if (m_change_flag){
            m_change_flag = false;
            return true;
        }
        return false;
    }

private:
    MsgT m_msg;
    MsgT m_first_msg;
    bool m_first_flag = false;
    bool m_change_flag = false;
    bool m_received = false;
    SubscriptionPtr m_sub;
};
