#ifndef _OFFBOARD_CTRL_NODE_H
#define _OFFBOARD_CTRL_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_tool.hpp"

class OffboardCtrlNode : public rclcpp::Node {
public:
    OffboardCtrlNode();
    void initialize();
//初始化
protected:
    void init_publisher();
    void init_subscription();
protected:
    void timer_callback();
    //发布offboard控制消息
    void publish_trajectory_setpoint();
private:
    rclcpp::TimerBase::SharedPtr m_timer;

    struct{
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint;
    } m_pub;

    struct{
        TopicListener<px4_msgs::msg::TrajectorySetpoint> target_setpoint;
        TopicListener<common_msgs::msg::ArmOffboardStatus> current_offboard_mode;
    } m_listener;
};

#endif
