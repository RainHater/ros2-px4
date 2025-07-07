#ifndef _PX4_HOLD_HEIGHT_TEST_NODE_H
#define _PX4_HOLD_HEIGHT_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>

class Px4HoldHeightTestNode : public rclcpp::Node {
public:
    Px4HoldHeightTestNode();
private:
    void init_publisher();
    void init_subscription();
    void timer_callback();
    //px4模式获取
    void px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg);
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布目标
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_set_point_pub;
    //设置offboard模式
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_set_offboard_mode_pub;
    //订阅当前飞控arm和offboard状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_sub;
    
    //当前飞控模式
    common_msgs::msg::ArmOffboardStatus m_px4_current_mode;
};

#endif

