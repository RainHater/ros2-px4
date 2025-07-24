#ifndef _SET_OFFBOARD_MODE_ACTION_H
#define _SET_OFFBOARD_MODE_ACTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "common_msgs/action/set_offboard_mode.hpp"
#include "common_msgs/msg/arm_offboard_status.hpp"
#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"

class SetOffboardModeAction : public rclcpp::Node {
public:
    using SetOffboardMode = common_msgs::action::SetOffboardMode;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SetOffboardMode>;
public:
    SetOffboardModeAction() 
        : rclcpp::Node("set_offboard_mode_action")
    {
        RCLCPP_INFO(get_logger(), "set_offboard_mode_action 节点启动...");
    }
    void initialize(){
        init_publisher();
        init_subscription();
        init_action();
    }
protected:
    void init_publisher(){
        m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_pub::SET_OFFBOARD_MODE, 10);
    }

    void init_subscription(){
        m_arm_offboard_status_listener.subscribe(
            shared_from_this(), 
            topic_sub::PX4_MODE_STATUS, 10
        );
    }

    void init_action(){
        m_action_srv = rclcpp_action::create_server<SetOffboardMode>(
            shared_from_this(),
            topic_srv::SET_OFFBOARD_MODE, 
            //处理请求
            [this](
                const rclcpp_action::GoalUUID &uuid, 
                std::shared_ptr<const SetOffboardMode::Goal> goal)
            {
                m_uuid = rclcpp_action::to_string(uuid);
                RCLCPP_INFO(get_logger(), "切换Offboard模式任务: %s, 接收的数据: arm=%d, offboard=%d", 
                                        m_uuid.c_str(), 
                                        goal->mode.arm,
                                        goal->mode.offboard);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            //处理取消请求
            [this](const std::shared_ptr<GoalHandle> goal_handle){
                (void)goal_handle;
                RCLCPP_INFO(get_logger(), "切换Offboard模式任务: %s, 已取消", m_uuid.c_str());
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            //接收并准备执行任务
            [this](const std::shared_ptr<GoalHandle> goal_handle){
                RCLCPP_INFO(get_logger(), "切换Offboard模式任务: %s 开始执行", m_uuid.c_str());
                std::thread{std::bind(&SetOffboardModeAction::execute, this, goal_handle)}.detach();
            }
        );
    }
    
    void execute(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        rclcpp::Rate rate(20);
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SetOffboardMode::Feedback>();
        auto result = std::make_shared<SetOffboardMode::Result>();

        while(!m_arm_offboard_status_listener.has_received()){
            if (!rclcpp::ok()){
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        auto target = goal->mode;
        auto &current = m_arm_offboard_status_listener.get_msg();

        RCLCPP_INFO(get_logger(), "切换offboard模式服务开始执行: "
            "目标arm: %d, 目标offboard: %d, "
            "当前arm: %d, 当前offboard: %d", 
            target.arm, target.offboard,
            current.arm, current.offboard
        );   

        while(rclcpp::ok()){
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "切换Offboard模式任务: %s 取消", m_uuid.c_str());
                return;
            }
            
            if (target.arm == current.arm && target.offboard == current.offboard) {   
                result->success = true;
                result->message = "成功切换模式";
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "成功切换模式");
                return;
            }
            common_msgs::msg::ArmOffboardStatus msg{};
            msg.offboard = target.offboard;
            msg.arm = target.arm;
            m_set_offboard_mode_pub->publish(msg);
            rate.sleep();
        }
    }
private:
    std::string m_uuid;
    rclcpp_action::Server<SetOffboardMode>::SharedPtr m_action_srv;
    //设置offboard模式
    rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr m_set_offboard_mode_pub;
    TopicListener<common_msgs::msg::ArmOffboardStatus> m_arm_offboard_status_listener;
};

#endif
