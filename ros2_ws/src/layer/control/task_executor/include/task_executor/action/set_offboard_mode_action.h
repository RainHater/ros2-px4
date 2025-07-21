#ifndef _SET_OFFBOARD_MODE_ACTION_H
#define _SET_OFFBOARD_MODE_ACTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/set_offboard_mode.hpp"
#include "common_msgs/msg/arm_offboard_status.hpp"
#include "utilities/topic_tool.hpp"

class SetOffboardModeService : public rclcpp::Node {
public:
    using SetOffboardMode = common_msgs::action::SetOffboardMode;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SetOffboardMode>;
public:
    SetOffboardModeService();
    void initialize();
protected:
    void init_publisher();
    void init_subscription();
    void init_action();
    void execute(const std::shared_ptr<GoalHandle> goal_handle);
private:
    std::string m_uuid;
    rclcpp_action::Server<SetOffboardMode>::SharedPtr m_action_srv;
    //设置offboard模式
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_set_offboard_mode_pub;
    TopicListener<common_msgs::msg::ArmOffboardStatus> m_arm_offboard_status_listener;
};

#endif
