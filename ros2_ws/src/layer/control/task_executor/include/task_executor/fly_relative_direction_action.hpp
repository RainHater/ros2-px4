#ifndef _FLY_RELATIVE_DIRECTION_ACTION_H
#define _FLY_RELATIVE_DIRECTION_ACTION_H

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "common_msgs/action/fly_relative_direction.hpp"
#include "common_msgs/msg/trajectory_set_point.hpp"
#include "api/set_offboard_mode_api.hpp"
#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;

class FlyRelativeDirectionAction : public rclcpp::Node{
public:
    using FlyRelative = common_msgs::action::FlyRelativeDirection;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FlyRelative>;
public:
    FlyRelativeDirectionAction()
        : Node("fly_relative_direction_action")
    {
        RCLCPP_INFO(get_logger(), "fly_relative_direction_action 节点启动...");
    }

    void initialize(){
        init_publisher();
        init_subscription();
        init_action();
    }
protected:
    void init_publisher(){
        m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_sub::TRAJECTORY_SETPOINT, 10);
    }

    void init_subscription(){
        m_vehicle_odometry_listener.subscribe(
        shared_from_this(), 
    topic_sub::VEHICLE_ODOMETRY, 10);
    }

    void init_action(){
        m_action_srv = rclcpp_action::create_server<FlyRelative>(
            shared_from_this(),
            topic_srv::FLY_RELATIVE_DIRECTION, 
            //处理请求
            [this](
                const rclcpp_action::GoalUUID &uuid, 
                std::shared_ptr<const FlyRelative::Goal> goal)
            {
                m_uuid = rclcpp_action::to_string(uuid);
                RCLCPP_INFO(get_logger(), "任务: %s, 接收的数据: forward=%f right=%f up=%f", 
                                        m_uuid.c_str(), 
                                        goal->forward, goal->right, goal->up);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            //处理取消请求
            [this](const std::shared_ptr<GoalHandle> goal_handle){
                (void)goal_handle;
                RCLCPP_INFO(get_logger(), "任务: %s, 已取消", m_uuid.c_str());
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            //接收并准备执行任务
            [this](const std::shared_ptr<GoalHandle> goal_handle){
                SetOffboardModeApi::Instance().send_goal(
                    shared_from_this(), 
                    ARM_ENABLE, 
                    POSITION, 
                    [this, goal_handle](){
                        RCLCPP_INFO(get_logger(), "任务: %s 开始执行", m_uuid.c_str());
                std::thread{std::bind(&FlyRelativeDirectionAction::execute, this, goal_handle)}.detach();
                    }
                );
            }
        );
    }

    void execute(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FlyRelative::Feedback>();
        auto result = std::make_shared<FlyRelative::Result>();
        auto &vehicle_odometry = m_vehicle_odometry_listener.get_msg();
        rclcpp::Rate rate(20);

        while(!m_vehicle_odometry_listener.has_received()){
            if (!rclcpp::ok()){
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        };

        tf2_tool::EulerAngles angles{};
        tf2_tool::get_euler_angles(vehicle_odometry, angles);
        float yaw = angles.yaw;
        float dx = goal->forward * std::cos(yaw) - goal->right * std::sin(yaw);
        float dy = goal->forward * std::sin(yaw) + goal->right * std::cos(yaw);
        float dz = -goal->up;

        float target[3] = {
            vehicle_odometry.position[0] + dx,
            vehicle_odometry.position[1] + dy,
            vehicle_odometry.position[2] + dz
        };
        
        while(rclcpp::ok()){
            auto current = vehicle_odometry.position;
            const float HORIZONTAL_DIST_THRESHOLD = 0.25f;
            const float VERTICAL_DIST_THRESHOLD = 0.26f;
            float dx = target[0] - current[0];
            float dy = target[1] - current[1];
            float dz = target[2] - current[2];
            float horizontal_dist = std::hypot(dx, dy);
            float vertical_dist = std::abs(dz);

            // float dist = std::sqrt(
            //     std::pow(current[0] - target[0], 2) +
            //     std::pow(current[1] - target[1], 2) +
            //     std::pow(current[2] - target[2], 2));
            feedback->traveled_distance = 0;
            goal_handle->publish_feedback(feedback);

            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "任务: %s 取消", m_uuid.c_str());
                return;
            }

            RCLCPP_INFO(get_logger(), "horizontal_dist: %f, vertical_dist: %f", 
                                        horizontal_dist, vertical_dist);
            
            bool arrive = (horizontal_dist < HORIZONTAL_DIST_THRESHOLD) && (vertical_dist < VERTICAL_DIST_THRESHOLD);
            if (arrive){
                result->success = true;
                result->message = "Reached target";
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "任务: %s 已完成", m_uuid.c_str());
                return;
            }

            common_msgs::msg::TrajectorySetPoint sp{};
            sp.position[0] = target[0];
            sp.position[1] = target[1];
            sp.position[2] = target[2];
            sp.yaw = yaw;

            // RCLCPP_INFO(get_logger(), "position[0]: %f, position[1]: %f, position[2]: %f", 
            //                             sp.position[0], sp.position[1], sp.position[2]);
            m_trajectory_setpoint_pub->publish(sp);
            rate.sleep();
        }
    }
private:
    std::string m_uuid;
    rclcpp_action::Server<FlyRelative>::SharedPtr m_action_srv;
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_setpoint_pub;
    TopicListener<px4_msgs::msg::VehicleOdometry> m_vehicle_odometry_listener;
};

#endif

