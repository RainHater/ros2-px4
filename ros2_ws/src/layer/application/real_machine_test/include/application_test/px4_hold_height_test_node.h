#ifndef _PX4_HOLD_HEIGHT_TEST_NODE_H
#define _PX4_HOLD_HEIGHT_TEST_NODE_H

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

enum TaskInfo{
    TASK1 = 0,
    TASK2,
    TASK3,
    TASK4,
    TASK5
};

class Px4HoldHeightTestNode : public rclcpp::Node {
public:
    Px4HoldHeightTestNode();
    void initialize();
private:
    void init_publisher();
    void init_subscription();
    void timer_callback();
    //px4模式获取
    void px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg);
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_1s_timer;
    //发布目标
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_set_point_pub;
    //设置offboard模式
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_set_offboard_mode_pub;
    //订阅当前飞控arm和offboard状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_sub;
    //订阅当前设定值
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_current_setpoing_sub;
    //当前设定值
    px4_msgs::msg::VehicleOdometry m_current_setpoint;
    //当前飞控模式
    common_msgs::msg::ArmOffboardStatus m_px4_current_mode;
    //滴答定时器
    uint64_t m_gobal_1s_timer;
    TaskInfo m_task_state;
};

#endif

