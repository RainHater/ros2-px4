#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"

#include "control_interface/mode_control.h"

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

class VisionTestNode : public rclcpp::Node{
public:
    VisionTestNode();
    void initialize();
protected:
    void init_pub();
    void init_sub();
protected:
    void task_loop();

private:
    enum FlyStep{
        IDLE,
        TO2Hover,
    }; 

    struct PubInfo{
        rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr arm_offboard_status;
    };

    struct SubInfo{
        TopicListener<common_msgs::msg::ArmOffboardStatus> arm_offboard_status;
    };

    struct PubMsgInfo{
        common_msgs::msg::ArmOffboardStatus arm_offboard_status;
    };

    struct InterfaceInfo{
        ModeControl mode_control;
    };
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    FlyStep m_fly;
    PubInfo m_pub;
    SubInfo m_sub;
    PubMsgInfo m_pub_msgs;
    InterfaceInfo m_interface;
};

#endif
