#ifndef _CONTROL_DESCENT_ACTION_H
#define _CONTROL_DESCENT_ACTION_H

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
    using GoalHandle = rclcpp_action::ServerGoalHandle<ControlledDescent>;
public:
    ControlledDescentAction();
    void initialize();
protected:
    void init_publisher();
    void init_subscription();
    void init_action();
    void execute(const std::shared_ptr<GoalHandle> goal_handle);
private:
    std::string m_uuid;
    rclcpp_action::Server<ControlledDescent>::SharedPtr m_action_srv;
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_setpoint_pub;
    TopicListener<px4_msgs::msg::VehicleLocalPosition> m_vehicle_local_position_listener;
};

#endif
