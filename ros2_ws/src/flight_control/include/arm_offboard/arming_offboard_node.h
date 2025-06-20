#ifndef _ARMING_OFFBOARD_NODE_H
#define _ARMING_OFFBOARD_NODE_H

#include <common_msgs/msg/detail/control_mode__struct.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

#define PX4_CUSTOM_MAIN_MODE_OFFBOARD 6

class ArmingOffboardNode : public rclcpp::Node {
public:
    ArmingOffboardNode();
protected:
    //发布一个 PX4 的 VehicleCommand 指令
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    //向 PX4 发布当前的 Offboard 控制模式(如位置控制、速度控制等)
    void publish_offboard_control_mode();
    //设置offboard模式回调函数
    void set_offboard_mode_callback(const common_msgs::msg::ControlMode msg);
    //发送 Arm(解锁)指令, 启动电机
    void arm();    
    //发送 Disarm(上锁)指令, 关闭电机
    void disarm();    
    //定时器回调函数
    void timer_callback();
private:
    //定时器，用于周期性发布消息
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布器：发布 vehicle_command 消息
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_vehicle_command_pub;
    //发布器：发布 offboard_control_mode 消息
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboard_control_mode_pub;
    //订阅设置offboard 
    rclcpp::Subscription<common_msgs::msg::ControlMode>::SharedPtr m_set_offboard_mode_sub;
    //当前offboard模式
    common_msgs::msg::ControlMode m_current_mode;
    //offboard setpoint 消息的计数器
    uint64_t m_offboard_setpoint_counter;
};

#endif

