#ifndef _POSITION_SETPOINT_NODE_H
#define _POSITION_SETPOINT_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PositionSetpointNode : public rclcpp::Node {
public:
    PositionSetpointNode();
protected:
    // 目标位置回调函数
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    //发布 trajectory setpoint 消息
    void publish_trajectory_setpoint();
private:
    //定时器，用于周期性发布消息
    rclcpp::TimerBase::SharedPtr m_timer;
    //订阅器：接收目标位置的消息
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_subscription;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_publisher;
    //目标位置
    geometry_msgs::msg::PoseStamped m_goal_pose;
};

#endif
