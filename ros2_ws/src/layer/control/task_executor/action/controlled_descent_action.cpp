#include "task_executor/action/controlled_descent_action.h"
#include "utilities/topic_name.hpp"
#include <rclcpp/logging.hpp>

ControlledDescentAction::ControlledDescentAction()
    : rclcpp::Node("controlled_descent_action")
{
    RCLCPP_INFO(get_logger(), "controlled_descent_action 节点启动...");
}

void ControlledDescentAction::initialize(){
    init_publisher();
    init_subscription();
    init_action();
}

void ControlledDescentAction::init_publisher(){
    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_sub::TRAJECTORY_SETPOINT, 10);
}

void ControlledDescentAction::init_subscription(){
    m_vehicle_local_position_listener.subscribe(
        shared_from_this(), 
    topic_sub::VEHICLE_LOCAL_POSITION, 10);
}

void ControlledDescentAction::init_action(){
    m_action_srv = rclcpp_action::create_server<ControlledDescent>(
        shared_from_this(),
        topic_srv::CONTROLLED_DESCENT,
        //处理请求
        [this](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ControlledDescent::Goal> goal)
        {   
            m_uuid = rclcpp_action::to_string(uuid);
             RCLCPP_INFO(get_logger(), "任务: %s, 接收的数据: speed=%f",
                                    m_uuid.c_str(), goal->speed);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle){
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "任务: %s, 已取消", m_uuid.c_str());
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle){
            RCLCPP_INFO(get_logger(), "任务: %s 开始执行", m_uuid.c_str());
            std::thread{std::bind(&ControlledDescentAction::execute, this, goal_handle)}.detach();
        }
    );
}

void ControlledDescentAction::execute(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ControlledDescent::Feedback>();
    auto result = std::make_shared<ControlledDescent::Result>();
    auto &local_position = m_vehicle_local_position_listener.get_msg();
    rclcpp::Rate rate(20);

    while(rclcpp::ok()){
        float dist_bottom = local_position.dist_bottom;
        RCLCPP_INFO(get_logger(), "dist_bottom: %f", dist_bottom);

        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Goal canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "任务: %s 取消", m_uuid.c_str());
            return;
        }

        if (dist_bottom < 0.05f){
            result->success = true;
            result->message = "Reached target";
            goal_handle->succeed(result);
            common_msgs::msg::TrajectorySetPoint msg{};
            msg.velocity[0] = 0.0;
            msg.velocity[1] = 0.0;
            msg.velocity[2] = 0.0;
            msg.yawspeed = 0.0;
            m_trajectory_setpoint_pub->publish(msg);
            RCLCPP_INFO(get_logger(), "任务: %s 已完成", m_uuid.c_str());
            return;
        }

        common_msgs::msg::TrajectorySetPoint msg{};
        msg.velocity[0] = 0.0;
        msg.velocity[1] = 0.0;
        msg.velocity[2] = goal->speed;
        msg.yawspeed = 0.0;
        m_trajectory_setpoint_pub->publish(msg);
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlledDescentAction>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
