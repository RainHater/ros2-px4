#ifndef _FLIGHT_MODE_MANAGER_NODE_H
#define _FLIGHT_MODE_MANAGER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>

struct Px4ModeInfo {
    //当前模式
    common_msgs::msg::ArmOffboardStatus current;
    //目标模式
    common_msgs::msg::ArmOffboardStatus target;
    //arm未解锁间隔
    int lock_interval_cnt;
};

class FlightModeManagerNode : public rclcpp::Node {
public:
    FlightModeManagerNode();
protected:
    void init_publisher();
    void init_subscription();
    void timer_callback();
    //设置offboard模式
    void set_px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus &msg);
    //获取飞控当前状态
    void px4_mode_status_broadcaster_callback(const px4_msgs::msg::VehicleStatus &msg);
    //发布一个 PX4 的 VehicleCommand 指令
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    //向PX4发布Offboard模式
    void publish_px4_offboard_mode();
    //向全局发布当前Offboard模式
    void publish_current_offboard_mode();
    //Arm解锁
    void arm();    
    //Arm上锁
    void disarm();
private:
    //定时器，用于周期性发布消息
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布 vehicle_command 消息
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_vehicle_command_pub;
    //发布 offboard_control_mode 消息
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboard_control_mode_pub;
    //发布当前Offboard模式消息
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_broadcaster_pub;
    //订阅设置offboard 
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_set_px4_mode_status_sub;
    //订阅px4飞控当前状态
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr m_px4_mode_status_broadcaster_sub;
    //飞控模式
    Px4ModeInfo m_px4_mode;
    //offboard setpoint 消息的计数器
    uint64_t m_offboard_setpoint_counter;

};

#endif

