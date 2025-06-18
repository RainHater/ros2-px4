#ifndef _POSITION_SETPOINT_NODE_H
#define _POSITION_SETPOINT_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/position_setpoint.hpp>

class PositionSetpointNode : public rclcpp::Node {
public:
    PositionSetpointNode();
protected:
    //发布 trajectory setpoint 消息
    void publish_trajectory_setpoint();
    //定时器回调
    void timer_callback();
    // 目标位置回调函数
    void target_position_callback(const common_msgs::msg::PositionSetpoint msg);
    //当前位置回调函数
    void current_position_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //订阅目标位置的消息
    rclcpp::Subscription<common_msgs::msg::PositionSetpoint>::SharedPtr m_target_position_sub;
    //订阅当前位置
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_current_position_sub;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    //目标位置
    common_msgs::msg::PositionSetpoint m_target_position;
    //当前位置
    common_msgs::msg::PositionSetpoint m_current_position;
    //最后一次yaw角度
    float m_last_yaw;
};

#endif
