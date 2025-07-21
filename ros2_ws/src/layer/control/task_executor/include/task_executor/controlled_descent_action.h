#ifndef _CONTROL_DESCENT_ACTION_H
#define _CONTROL_DESCENT_ACTION_H

#include <common_msgs/action/detail/controlled_descent__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "common_msgs/action/controlled_descent.hpp"
#include "common_msgs/msg/trajectory_set_point.hpp"
#include "utilities/topic_tool.hpp"

class ControlledDescentAction : public rclcpp::Node{
public:
    using ControlledDescent = common_msgs::action::ControlledDescent;
public:
    ControlledDescentAction();
    void initialize();
protected:
    void init_action();
private:
    std::string m_uuid;
    rclcpp_action::Server<ControlledDescent>::SharedPtr m_action_srv;
};

#endif
