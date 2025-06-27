#ifndef _MISSION_PLANNER_NODE_H
#define _MISSION_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/target_gps.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/action/navigate_to_gps.hpp>
#include <control/navigation_controller.h>

using NavigateToGPS = common_msgs::action::NavigateToGPS;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToGPS>;

enum TaskStatus {
    WAIT_FOR_ARM_AND_OFFBOARD = 0,
    FLY_TO_READY_POSITION,
    FLY_TO_READY_POSITION_AND_LAND,  // 如果需要在准备点降落
    FLY_TO_GPS_TARGET,
    FLY_TO_GPS_TARGET_AND_LAND,      // 到GPS目标点后降落
};

class MissionPlanner : public rclcpp::Node {
public:
    MissionPlanner();
    void init();
protected:
    void init_publisher();
    void init_subscription();
    void init_callback();
    void timer_callback();
    //px4模式获取
    void px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //发布GPS位置
    rclcpp::Publisher<common_msgs::msg::TargetGps>::SharedPtr m_trajectory_setpoint_pub;
    //订阅当前飞控arm和offboard状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_sub;
    //目标经纬度服务
    std::shared_ptr<NavigationController> m_nav_controller;
    //当前任务状态
    TaskStatus m_current_task_status = WAIT_FOR_ARM_AND_OFFBOARD;
};

#endif
