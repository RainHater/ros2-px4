#include "control_interface/mode_control.h"
#include <rclcpp/logger.hpp>

constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

ModeControl::ModeControl(){
    m_log_name = "切换飞控模式(mode_control.cpp)";
}

void ModeControl::unlock(
    uint8_t arm_mode,
    uint8_t offboard_mode,
    common_msgs::msg::ArmOffboardStatus current,
    common_msgs::msg::ArmOffboardStatus &pub_msg)
{   
    set_mode(
        arm_mode, 
        offboard_mode, 
        current, 
        pub_msg, 
        "解锁"
    );
}

void ModeControl::locked(
    common_msgs::msg::ArmOffboardStatus current,
    common_msgs::msg::ArmOffboardStatus &pub_msg)
{   
    set_mode(
        ARM_DISABLED, 
        OFFBOARD_DISABLED, 
        current, 
        pub_msg, 
        "上锁"
    );
}

bool ModeControl::wait_busy(){
    return !m_states.is_busy;
}
