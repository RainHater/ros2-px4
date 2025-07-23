#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utilities/task_scheduler.hpp"
#include "fly_relative_direction_api.hpp"
#include "set_offboard_mode_api.hpp"
#include "controlled_descent_api.hpp"

constexpr auto ARMING_STATE_ARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
constexpr auto ARMING_STATE_DISARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_DISARMED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto OFFBOARD_NOT_ACTIVE = common_msgs::msg::ArmOffboardStatus::OFFBOARD_NOT_ACTIVE;

class SimpleIndoorControlTask : public rclcpp::Node{
public:
    SimpleIndoorControlTask()
        : rclcpp::Node("simple_indoor_control_task")
    {
        RCLCPP_INFO(get_logger(), "simple_indoor_control_task 节点启动...");
    }

    void initialize(){
        using namespace task_tool;

        m_scheduler.add_task(
            [this](auto &flag){
                SetOffboardModeApi::Instance().send_goal(
                    shared_from_this(), 
                    ARMING_STATE_ARMED, 
                    POSITION, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.0, 0.0, 0.5, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                FlyRelativeDirectionApi::Instance().send_goal(
                    shared_from_this(), 
                    0.5, 0.0, 0, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务已完成!");
                    }
                );
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                ControlledDescentApi::Instance().send_goal(
                    shared_from_this(), 0.25, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务已完成!");
                    }
                );    
            }
        );

        m_scheduler.add_task(
            [this](auto &flag){
                SetOffboardModeApi::Instance().send_goal(
                    shared_from_this(), 
                    ARMING_STATE_DISARMED, 
                    OFFBOARD_NOT_ACTIVE, 
                    [this, flag](){
                        *flag = true;
                        RCLCPP_INFO(get_logger(), "任务已完成!");
                    }
                );
            }
        );

        m_timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimpleIndoorControlTask::timer_callback, this));
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
