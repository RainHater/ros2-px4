#ifndef _MISSION_PLANNER_NODE_H
#define _MISSION_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/action/navigate_to_gps.hpp>
#include <common_msgs/msg/tracking_feedback.hpp>
#include <common_msgs/msg/pid_debug.hpp>
#include "mission_planner/IncrementalPID.hpp"
#include "utilities/topic_tool.hpp"

enum TaskStatus {
    WAIT_FOR_ARM_AND_OFFBOARD = 0,
    FLY_TO_READY_POSITION,
    FLY_TO_READY_POSITION_AND_LAND,
    FLY_TO_GPS_TARGET,
    SWITCH_TO_OFFBOARD_VELOCITY_MODE,  
    VISUAL_TRACKING,
    LANDING, 
};

class MissionPlanner : public rclcpp::Node {
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
    // using CommonPrecisionLand = common_msgs::action::PrecisionLand;
public:
    MissionPlanner();
    void initialized();
protected:
    void init_publisher();
    void init_subscription();
    void init_client();
    void timer_callback();
    //px4模式获取
    void px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //设置offboard模式
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_set_offboard_mode_pub;
    //发布目标
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_set_point_pub;
    //发布PIDdebug波形显示
    rclcpp::Publisher<common_msgs::msg::PidDebug>::SharedPtr m_pid_viewer_pub;
    //订阅当前飞控arm和offboard状态
    rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_px4_mode_status_sub;
    //nav导航客户端
    rclcpp_action::Client<NavigateToGPS>::SharedPtr m_nav_client;
    //视觉消息监听
    TopicListener<common_msgs::msg::TrackingFeedback> m_tracking_feedback_listener;

    //当前任务状态
    TaskStatus m_current_task_status = WAIT_FOR_ARM_AND_OFFBOARD;

    IncrementalPID m_pid_x;
    IncrementalPID m_pid_y;
};

#endif
