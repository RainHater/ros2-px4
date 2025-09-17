#ifndef _FLIGHT_MODE_MANAGER_NODE_H
#define _FLIGHT_MODE_MANAGER_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_name.h"

#include <stdint.h>
#include <chrono>
#include <iostream>

typedef enum {
    IDLE,                       //空闲状态，未开始初始化流程
    SENDING_SETPOINT,           //发送初始 setpoint（位置/速度等）给飞控，准备进入 Offboard 模式
    SETTING_OFFBOARD,           //发送切换到 Offboard 模式的命令
    WAITING_OFFBOARD_CONFIRM,   //等待飞控确认已成功切换到 Offboard 模式
    ARMING,                     //发送解锁（解锁电机）命令，准备起飞
    WAITING_ARM_CONFIRM,        //等待飞控确认已成功解锁
    READY,                      //已解锁且处于 Offboard 模式，准备执行飞行任务
    FAILED                      //初始化失败，可能需要重试或错误处理
} FlightInitState;

class FlightModeManagerNode : public rclcpp::Node {
public:
    FlightModeManagerNode();
    void initialize();
protected:
    void initPub();
    void initSub();
protected:
    //解锁状态机
    void armAndSetOffboard();
    //向PX4发布Offboard模式
    void pubPx4OffboardMode();
    //向全局发布当前Offboard模式
    void pubCurrentMode();

    //消息回调函数
    void battStatusCallback(const std::shared_ptr<px4_msgs::msg::BatteryStatus> msg);
    void px4OffboardCallback(const std::shared_ptr<px4_msgs::msg::VehicleStatus> msg);
    void setOffboardCallback(const std::shared_ptr<common_msgs::msg::ArmOffboardStatus> msg);
//工具函数
private:
    //Arm解锁
    void arm();  
    //Arm上锁
    void disarm();
    //发布一个 PX4 的 VehicleCommand 指令
    void pubVehicleCommand(
        uint16_t command,
        float param1 = 0.0,
        float param2 = 0.0,
        float param3 = 0.0,
        float param4 = 0.0,
        float param5 = 0.0,
        float param6 = 0.0,
        float param7 = 0.0
    );
protected:
    struct PubInfo{
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offb_ctrl_mode;
        rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr px4_mode_broad;
    };

    struct SubInfo{
        rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr batt_status;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_offb;
        rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr set_offb;
    };

    struct ModeInfo{
        uint64_t setpoint_counter;
        uint8_t cur_px4_offb;
        uint8_t cur_px4_arm;
        uint16_t cur_offb;
        uint8_t cur_arm;
        uint16_t tar_offb;
        uint8_t tar_arm;
    };

private:
    rclcpp::TimerBase::SharedPtr m_timer;
    FlightInitState m_flight_state;
    PubInfo m_pub;  
    SubInfo m_sub;
    ModeInfo m_offb_mode;
};

#endif
