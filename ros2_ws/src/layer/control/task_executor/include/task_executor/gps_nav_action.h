#ifndef _GPS_NAV_ACTION_H
#define _GPS_NAV_ACTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/msg/arm_offboard_status.hpp>
#include <common_msgs/action/navigate_to_gps.hpp>
#include <common_msgs/srv/transform_gps_to_local.hpp>

class GpsNavAction : public rclcpp::Node{
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
    using SrvTransformGpsToLocal = common_msgs::srv::TransformGpsToLocal;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToGPS>;
public:
    GpsNavAction();
    void initialize();
protected:
    void init_publisher();
    void init_action();
    void init_client();
    //执行导航任务逻辑
    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);
    //请求转换函数
    bool request_local_target(
        const std::shared_ptr<SrvTransformGpsToLocal::Request> request, 
        std::shared_ptr<SrvTransformGpsToLocal::Response> &response);
private:
    std::string m_uuid;
    //订阅目标位置的消息
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_target_setpoint_pub;
    //飞到目标经纬度动作
    rclcpp_action::Server<NavigateToGPS>::SharedPtr m_action_srv;
    //请求坐标转换客户端
    rclcpp::Client<common_msgs::srv::TransformGpsToLocal>::SharedPtr m_gps_transform_client;
};

#endif
