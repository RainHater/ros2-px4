#ifndef _OFFBOARD_CTRL_NODE_H
#define _OFFBOARD_CTRL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/msg/task_status.hpp>

class OffboardCtrlNode : public rclcpp::Node {
public:
    OffboardCtrlNode();
protected:
    //定时器回调
    void timer_callback();
    //发布offboard控制消息
    void publish_trajectory_setpoint();
    //判断数据是否有效
    bool non_zero3(const std::array<float, 3>& v);
    //填充位置控制的数据
    void fill_position(
        const px4_msgs::msg::VehicleOdometry &target, 
        px4_msgs::msg::TrajectorySetpoint &msg);
    //填充速度控制的数据
    void fill_velocity(
        const px4_msgs::msg::VehicleOdometry &target, 
        px4_msgs::msg::TrajectorySetpoint &msg);
    // //position模式
    // void offboard_position_mode(px4_msgs::msg::TrajectorySetpoint &msg);
    // //velocity模式
    // void offboard_velocity_mode(px4_msgs::msg::TrajectorySetpoint &msg);
    // bool is_at_target_position(float dx, float dy, float dz);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    //发布当前任务状态
    rclcpp::Publisher<common_msgs::msg::TaskStatus>::SharedPtr m_task_status_pub;
    //订阅目标位置的消息
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_trajectory_setpoint_sub;
    //订阅当前位置
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_current_setpoint_sub;
    //订阅当前offboard模式状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_current_offboard_mode_sub;
    //目标位置
    px4_msgs::msg::VehicleOdometry m_target_setpoint;
    //当前位置
    px4_msgs::msg::VehicleOdometry m_current_setpoint;
    //控制缓存
    px4_msgs::msg::TrajectorySetpoint m_traj_msg_cache;
    //当前offboard模式状态
    common_msgs::msg::ArmOffboardStatus px4_mode_status_broadcaster;
    //当前任务状态
    common_msgs::msg::TaskStatus m_task_status;
    //最后一次yaw角度
    float m_last_yaw;
};

#endif
