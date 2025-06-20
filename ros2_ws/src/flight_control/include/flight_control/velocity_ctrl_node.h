#ifndef _VELOCITY_CTRL_NODE_H
#define _VELOCITY_CTRL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <common_msgs/msg/position_setpoint.hpp>

class VelocityCtrlNode : public rclcpp::Node {
public:
    VelocityCtrlNode();
protected:
    //定时器回调
    void timer_callback();
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //当前速度
    geometry_msgs::msg::Twist m_current_velocity;
};

#endif
