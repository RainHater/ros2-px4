#include "control_interface/mode_control.h"

constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

ModeControl::ModeControl()
 : m_log(rclcpp::get_logger("切换飞控模式(mode_control.cpp)"))
 , is_busy(false)
{
    
}

void ModeControl::unlock(
    uint8_t tar_arm, uint16_t tar_offb,
    uint8_t cur_arm, uint16_t cur_offb,
    common_msgs::msg::ArmOffboardStatus &pub_msg
) {   
    setMode(
        "解锁",
        tar_arm, 
        tar_offb,
        cur_arm,
        cur_offb,
        pub_msg
    );
}

void ModeControl::locked(
    uint8_t cur_arm, uint16_t cur_offb,
    common_msgs::msg::ArmOffboardStatus &pub_msg)
{   
    setMode(
        "上锁",
        ARM_DISABLED, 
        OFFBOARD_DISABLED, 
        cur_arm, 
        cur_offb,
        pub_msg
    );
}

void ModeControl::setMode(
    const std::string &action_desc,
    uint8_t tar_arm, uint16_t tar_offb,
    uint8_t cur_arm, uint16_t cur_offb,
    common_msgs::msg::ArmOffboardStatus &pub_msg
    
) {
    if (!is_busy) {
        pub_msg.arm = tar_arm;
        pub_msg.offboard = tar_offb;
        RCLCPP_INFO(m_log, 
            "[%s] 目标 arm: %d, offboard: %d, 当前 arm: %d, offboard: %d",
            action_desc.c_str(), tar_arm, tar_offb,
            cur_arm, cur_offb);
        is_busy = true;
    }

    if (cur_arm == tar_arm && cur_offb == tar_offb) {
        is_busy = false;
        RCLCPP_INFO(m_log, "[%s] 模式切换成功!", action_desc.c_str());
    }
}

bool ModeControl::waitBusy(){
    return !is_busy;
}
