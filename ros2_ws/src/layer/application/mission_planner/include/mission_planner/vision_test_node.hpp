#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <rclcpp/rclcpp.hpp>

#include "utilities/task_scheduler.hpp"
#include "fly_relative_direction_api.hpp"
#include "set_offboard_mode_api.hpp"
#include "controlled_descent_api.hpp"

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

class VisionTestNode : public rclcpp::Node{
public:
    VisionTestNode()
        : rclcpp::Node("vision_test_node")
    {
        RCLCPP_INFO(get_logger(), "vision_test_node 节点启动...");
    }

    void initialize(){
        using namespace task_tool;

        m_scheduler.add_task(
            [this](auto &flag){
                SetOffboardModeApi::Instance().send_goal(
                    shared_from_this(), 
                    ARM_ENABLE, 
                    POSITION, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务1已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.0, 0.0, 0.5, 0.0,
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务2已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.0, 0.0, 0, 45, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务3已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.0, 0.0, 0, -90, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务4已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.0, 0.0, 0, 90, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务5已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.0, 0.0, 0, -90, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务6已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                ControlledDescentApi::Instance().send_goal(
                    shared_from_this(), 0.3, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务7已完成!");
                    }
                );    
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                SetOffboardModeApi::Instance().send_goal(
                    shared_from_this(), 
                    ARM_DISABLED, 
                    OFFBOARD_DISABLED, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务已完成!");
                    }
                );
            }
        );

        m_timer = create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&VisionTestNode::timer_callback, this));
    }

    void timer_callback(){
        m_scheduler.run();
        if (m_scheduler.is_done()) {
            RCLCPP_INFO(get_logger(), "所有任务完成");
            m_timer->cancel();
        }
    }
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    task_tool::TaskScheduler m_scheduler;
};

#endif
