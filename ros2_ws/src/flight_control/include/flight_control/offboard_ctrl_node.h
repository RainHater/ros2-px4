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
    //定时器回调
    void timer_callback();
    // 目标位置回调函数
    void trajectory_setpoint_callback(const common_msgs::msg::TrajectorySetPoint msg);
    //当前位置回调函数
    void current_position_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    //当前offboard模式状态回调函数
    void current_offboard_callback(const common_msgs::msg::ArmOffboardStatus msg);
    //发布offboard控制消息
    void publish_trajectory_setpoint();
    //position模式
    void offboard_position_mode(px4_msgs::msg::TrajectorySetpoint &msg);
    //velocity模式
    void offboard_velocity_mode(px4_msgs::msg::TrajectorySetpoint &msg);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    //订阅目标位置的消息
    rclcpp::Subscription<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_setpoint_sub;
    //订阅当前位置
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_current_position_sub;
    //订阅当前offboard模式状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_current_offboard_mode_sub;
    //目标位置
    common_msgs::msg::TrajectorySetPoint m_trajectory_setpoint;
    //当前位置
    common_msgs::msg::TrajectorySetPoint m_current_setpoint;
    //当前offboard模式状态
    common_msgs::msg::ArmOffboardStatus px4_mode_status_broadcaster;
    //最后一次yaw角度
    float m_last_yaw;
};

#endif
