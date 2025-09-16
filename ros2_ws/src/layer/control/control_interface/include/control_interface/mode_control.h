#ifndef _MODE_CONTROL_H
#define _MODE_CONTROL_H

#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_name.hpp"

class ModeControl{
public:
    ModeControl();
    //切换模式或解锁
    void unlock(
        uint8_t tar_arm, uint16_t tar_offb,
        uint8_t cur_arm, uint16_t cur_offb,
        common_msgs::msg::ArmOffboardStatus &pub_msg
    );
    //上锁
    void locked(
        uint8_t cur_arm, uint16_t cur_offb,
        common_msgs::msg::ArmOffboardStatus &pub_msg
    );
    //等待切换
    bool waitBusy();
protected:
    //设置模式
    void setMode(
        const std::string &action_desc,
        uint8_t tar_arm, uint16_t tar_offb,
        uint8_t cur_arm, uint16_t cur_offb,
        common_msgs::msg::ArmOffboardStatus &pub_msg
    );
private:
    struct StatusBits{
        bool is_busy = false;
    };
private:
    rclcpp::Logger m_log;
    bool is_busy;
};

#endif
