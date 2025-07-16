#ifndef _OFFBOARD_CTRL_NODE_H
#define _OFFBOARD_CTRL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>

class OffboardCtrlNode : public rclcpp::Node {
public:
    OffboardCtrlNode();
protected:
    void init_publisher();
    void init_subscription();
    void timer_callback();
    //发布offboard控制消息
    void publish_trajectory_setpoint();
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    //订阅目标位置的消息
    rclcpp::Subscription<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_target_setpoint_sub;
    //订阅当前offboard模式状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_current_offboard_mode_sub;
    
    //目标位置
    common_msgs::msg::TrajectorySetPoint m_target_setpoint;
    //当前offboard模式状态
    common_msgs::msg::ArmOffboardStatus m_current_offboard_mode;
};

#endif
