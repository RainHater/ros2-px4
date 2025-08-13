#include "control_interface/mode_control.h"

constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

ModeControl::ModeControl()
 : m_log(rclcpp::get_logger("切换飞控模式(mode_control.cpp)"))
{
    
}

void ModeControl::unlock(
    uint8_t arm_mode,
    uint16_t offboard_mode,
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

void ModeControl::set_mode(
    uint8_t target_arm,
    uint16_t target_offboard,
    const common_msgs::msg::ArmOffboardStatus &current,
    common_msgs::msg::ArmOffboardStatus &pub_msg,
    const std::string &action_desc)
{
    if (!m_states.is_busy) {
        pub_msg.arm = target_arm;
        pub_msg.offboard = target_offboard;
        RCLCPP_INFO(m_log, 
            "[%s] 目标 arm: %d, offboard: %d, 当前 arm: %d, offboard: %d",
            action_desc.c_str(), target_arm, target_offboard,
            current.arm, current.offboard);
        m_states.is_busy = true;
    }

    if (current.arm == target_arm && current.offboard == target_offboard) {
        m_states.is_busy = false;
        RCLCPP_INFO(m_log, "[%s] 模式切换成功!", action_desc.c_str());
    }
}

bool ModeControl::wait_busy(){
    return !m_states.is_busy;
}
