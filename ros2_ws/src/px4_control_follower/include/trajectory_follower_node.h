#ifndef TRAJECTORY_FOLLOWER_NODE_H
#define TRAJECTORY_FOLLOWER_NODE_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

class TrajectoryFollower : public rclcpp::Node
{
public:
    TrajectoryFollower();
private:
    //定时器，用于周期性发布消息
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布器：发布 offboard_control_mode 消息
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboard_control_mode_publisher;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_publisher;
    //发布器：发布 vehicle_command 消息
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_vehicle_command_publisher;
    //订阅器：接收目标位置的消息
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_subscription;
    //存储时间戳的原子变量
    std::atomic<uint64_t> m_timestamp;
    //offboard setpoint 消息的计数器
    uint64_t m_offboard_setpoint_counter;
    //目标位置
    geometry_msgs::msg::PoseStamped m_goal_pose;

    //发布 offboard control mode 消息
    void publish_offboard_control_mode();
    //发布 trajectory setpoint 消息
    void publish_trajectory_setpoint();
    //发布 vehicle command 消息
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    //发送 arm 命令
    void arm();
    //发送 disarm 命令
    void disarm();
    // 目标位置回调函数
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif
