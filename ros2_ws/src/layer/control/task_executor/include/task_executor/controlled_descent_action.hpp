#ifndef _CONTROL_DESCENT_ACTION_H
#define _CONTROL_DESCENT_ACTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "common_msgs/action/controlled_descent.hpp"
#include "common_msgs/msg/trajectory_set_point.hpp"
#include "api/set_offboard_mode_api.hpp"
#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;

class ControlledDescentAction : public rclcpp::Node{
public:
    using ControlledDescent = common_msgs::action::ControlledDescent;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ControlledDescent>;
public:
    ControlledDescentAction()
        : rclcpp::Node("controlled_descent_action")
    {
        RCLCPP_INFO(get_logger(), "controlled_descent_action 节点启动...");
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
        m_vehicle_local_position_listener.subscribe(
        shared_from_this(), 
    topic_sub::VEHICLE_LOCAL_POSITION, 10);
    }
    void init_action(){
        m_action_srv = rclcpp_action::create_server<ControlledDescent>(
            shared_from_this(),
            topic_srv::CONTROLLED_DESCENT,
            //处理请求
            [this](
                const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const ControlledDescent::Goal> goal)
            {   
                m_uuid = rclcpp_action::to_string(uuid);
                RCLCPP_INFO(get_logger(), "降落任务: %s, 接收的数据: speed=%f",
                                        m_uuid.c_str(), goal->speed);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<GoalHandle> goal_handle){
                (void)goal_handle;
                RCLCPP_INFO(get_logger(), "降落任务: %s, 已取消", m_uuid.c_str());
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> goal_handle){
                SetOffboardModeApi::Instance().send_goal(
                    shared_from_this(), 
                    ARM_ENABLE, 
                    VELOCITY, 
                    [this, goal_handle](){
                        RCLCPP_INFO(get_logger(), "降落任务: %s 开始执行", m_uuid.c_str());
                        std::thread{std::bind(&ControlledDescentAction::execute, this, goal_handle)}.detach();
                    }
                );
            }
        );
    }
    void execute(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ControlledDescent::Feedback>();
        auto result = std::make_shared<ControlledDescent::Result>();
        auto &local_position = m_vehicle_local_position_listener.get_msg();
        rclcpp::Rate rate(20);

        while(rclcpp::ok()){
            auto vz = local_position.vz;
            auto dist_bottom = local_position.dist_bottom;
            auto dist_bottom_valid = local_position.dist_bottom_valid;
            RCLCPP_INFO(get_logger(), "vz %f, dist_bottom: %f, dist_bottom_valid: %d", 
                vz, dist_bottom, dist_bottom_valid);

            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "降落任务: %s 取消", m_uuid.c_str());
                return;
            }

            if (dist_bottom_valid && dist_bottom < 0.1f && std::abs(vz) < 0.2f) {
                result->success = true;
                result->message = "Reached target";
                goal_handle->succeed(result);
                drop_send(0.0);
                RCLCPP_INFO(get_logger(), "降落任务: %s 已完成", m_uuid.c_str());
                return;
            }

            drop_send(goal->speed);
            rate.sleep();
        }
    }
protected:
    //发送下降消息
    void drop_send(float speed){
        common_msgs::msg::TrajectorySetPoint msg{};
        msg.velocity[0] = 0.0;
        msg.velocity[1] = 0.0;
        msg.velocity[2] = speed;
        msg.yawspeed = 0.0;
        m_trajectory_setpoint_pub->publish(msg);
    }
private:
    std::string m_uuid;
    rclcpp_action::Server<ControlledDescent>::SharedPtr m_action_srv;
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_setpoint_pub;
    TopicListener<px4_msgs::msg::VehicleLocalPosition> m_vehicle_local_position_listener;
};

#endif
