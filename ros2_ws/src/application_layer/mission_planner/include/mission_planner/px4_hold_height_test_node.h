#ifndef _PX4_HOLD_HEIGHT_TEST_NODE_H
#define _PX4_HOLD_HEIGHT_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>

class Px4HoldHeightTestNode : public rclcpp::Node {
public:
    Px4HoldHeightTestNode();
private:
    void init_publisher();
    void timer_callback();
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布目标
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_set_point_pub;
};

#endif

