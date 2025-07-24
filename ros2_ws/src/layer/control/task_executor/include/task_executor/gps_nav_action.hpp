#ifndef _GPS_NAV_ACTION_H
#define _GPS_NAV_ACTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "common_msgs/msg/trajectory_set_point.hpp"
#include "common_msgs/msg/arm_offboard_status.hpp"
#include "common_msgs/action/navigate_to_gps.hpp"
#include "common_msgs/srv/transform_gps_to_local.hpp"
#include "utilities/topic_name.hpp"

class GpsNavAction : public rclcpp::Node{
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
    using SrvTransformGpsToLocal = common_msgs::srv::TransformGpsToLocal;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToGPS>;
public:
    GpsNavAction()
        : Node("gps_nav_action")
    {
        RCLCPP_INFO(get_logger(), "gps_nav_action 节点启动...");
    }

    void initialize(){
        init_publisher();
        init_action();
        init_client();
    }
protected:
    void init_publisher(){
        m_target_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_pub::TRAJECTORY_SETPOINT, 10);
    }

    void init_action(){
        m_action_srv = rclcpp_action::create_server<NavigateToGPS>(
            shared_from_this(),
            topic_srv::NAVIGATE_TO_GPS, 
            //处理导航目标请求
            [this](
                const rclcpp_action::GoalUUID &uuid, 
                std::shared_ptr<const NavigateToGPS::Goal> goal)
            {
                m_uuid = rclcpp_action::to_string(uuid);
                RCLCPP_INFO(get_logger(), "任务: %s, 接收的数据: lat=%f lon=%f alt=%f", 
                                        m_uuid.c_str(), 
                                        goal->lat, goal->lon, goal->alt);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            //处理取消导航请求
            [this](const std::shared_ptr<GoalHandleNavigate> goal_handle){
                (void)goal_handle;
                RCLCPP_INFO(get_logger(), "任务: %s, 已取消", m_uuid.c_str());
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            //接收并准备执行导航任务
            [this](const std::shared_ptr<GoalHandleNavigate> goal_handle){
                RCLCPP_INFO(get_logger(), "任务: %s 开始执行", m_uuid.c_str());
                std::thread{std::bind(&GpsNavAction::execute, this, goal_handle)}.detach();
            }
        );
    }

    void init_client(){
        m_gps_transform_client = create_client<SrvTransformGpsToLocal>(
        topic_cli::TRANSFORM_GPS_TO_LOCAL);
    }

    //执行导航任务逻辑
    void execute(
        const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        auto result = std::make_shared<NavigateToGPS::Result>();
        auto feedback = std::make_shared<NavigateToGPS::Feedback>();
        auto goal = goal_handle->get_goal();

        while(rclcpp::ok()){
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Cancelled";
                goal_handle->canceled(result);
                return;
            }

            auto request = std::make_shared<SrvTransformGpsToLocal::Request>();
            auto response = std::make_shared<SrvTransformGpsToLocal::Response>();
            request->latitude = goal->lat;
            request->longitude = goal->lon;
            request->altitude = goal->alt;
            if (!request_local_target(request, response))
                continue;
            
            common_msgs::msg::TrajectorySetPoint msg{};

            msg.position[0] = response->x;
            msg.position[1] = response->y;
            msg.position[2] = response->z;
            msg.yaw = response->yaw;
            m_target_setpoint_pub->publish(msg);

            feedback->current_latitude = response->lat;
            feedback->current_longitude = response->lon;
            feedback->current_altitude = response->alt;
            feedback->distance_remaining = 0;

            goal_handle->publish_feedback(feedback);

            if (response->arrive)
                break;
        }

        result->success = true;
        result->message = "Arrived at target";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "任务: %s 已完成", m_uuid.c_str());
    }
    //请求转换函数
    bool request_local_target(
        const std::shared_ptr<SrvTransformGpsToLocal::Request> request, 
        std::shared_ptr<SrvTransformGpsToLocal::Response> &response)
    {
        if (!m_gps_transform_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "GPS transform service not available.");
            return false;
        }

        auto future = m_gps_transform_client->async_send_request(request);
        response = future.get();
        return true;
    }
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
