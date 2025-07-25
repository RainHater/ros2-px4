#ifndef _MODE_CONTROL_H
#define _MODE_CONTROL_H

#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"

class ModeControl{
public:
    ModeControl();
    void unlock(
        uint8_t arm_mode,
        uint8_t offboard_mode,
        common_msgs::msg::ArmOffboardStatus current,
        common_msgs::msg::ArmOffboardStatus &pub_msg
    );
    void locked(
        common_msgs::msg::ArmOffboardStatus current,
        common_msgs::msg::ArmOffboardStatus &pub_msg
    );
    bool wait_busy();
protected:
    void set_mode(
        uint8_t target_arm,
        uint8_t target_offboard,
        const common_msgs::msg::ArmOffboardStatus &current,
        common_msgs::msg::ArmOffboardStatus &pub_msg,
        const std::string &action_desc
    );
private:
    struct StatusBits{
        bool is_busy = false;
    };
private:
    std::string m_log_name;
    StatusBits m_states;
};

#endif
