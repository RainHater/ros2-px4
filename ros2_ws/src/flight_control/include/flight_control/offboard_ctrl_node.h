#ifndef _OFFBOARD_CTRL_NODE_H
#define _OFFBOARD_CTRL_NODE_H

#include <common_msgs/msg/detail/trajectory_set_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/action/navigate_to_gps.hpp>

using NavigateToGPS = common_msgs::action::NavigateToGPS;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToGPS>;

class OffboardCtrlNode : public rclcpp::Node {
public:
    OffboardCtrlNode();
protected:
    //定时器回调
    void timer_callback();
    //目标位置订阅
    void target_setpoint_callback(const common_msgs::msg::TrajectorySetPoint::SharedPtr msg);
    //处理导航目标请求
    rclcpp_action::GoalResponse nav_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToGPS::Goal> goal);
    //处理取消导航请求
    rclcpp_action::CancelResponse nav_handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle);
    //接收并准备执行导航任务
    void nav_handle_accepted(
        const std::shared_ptr<GoalHandleNavigate> goal_handle);
    //执行导航任务逻辑
    void nav_execute(
        const std::shared_ptr<GoalHandleNavigate> goal_handle);
    //发布offboard控制消息
    void publish_trajectory_setpoint();
    //判断数据是否有效
    bool non_zero3(const std::array<float, 3>& v);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //飞到目标经纬度动作
    rclcpp_action::Server<NavigateToGPS>::SharedPtr m_action_nav_server;
    //发布器：发布 trajectory_setpoint 消息
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    //订阅目标位置的消息
    rclcpp::Subscription<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_target_setpoint_sub;
    //订阅当前offboard模式状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_current_offboard_mode_sub;
    //目标位置
    common_msgs::msg::TrajectorySetPoint m_target_setpoint;
    //控制缓存
    px4_msgs::msg::TrajectorySetpoint m_traj_msg_cache;
    //当前offboard模式状态
    common_msgs::msg::ArmOffboardStatus px4_mode_status_broadcaster;
};

#endif
