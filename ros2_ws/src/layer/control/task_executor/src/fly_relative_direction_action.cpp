#include "task_executor/fly_relative_direction_action.h"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"

FlyRelativeDirectionAction::FlyRelativeDirectionAction()
    : Node("fly_relative_direction_action")
{
    RCLCPP_INFO(get_logger(), "Starting fly_relative_direction_action follower node...");
}

void FlyRelativeDirectionAction::initialize(){
    init_publisher();
    init_subscription();
    init_action();
}

void FlyRelativeDirectionAction::init_publisher(){
    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_sub::TRAJECTORY_SETPOINT, 10);
}

void FlyRelativeDirectionAction::init_subscription(){
    m_vehicle_odometry_listener.subscribe(
        shared_from_this(), 
    topic_sub::VEHICLE_ODOMETRY, 10);
    
    // m_vehicle_attitude_listener.subscribe(
    //     shared_from_this(), 
    // topic_sub::V, 10);
}

void FlyRelativeDirectionAction::init_action(){
    m_action_srv = rclcpp_action::create_server<FlyRelative>(
        this,
        topic_srv::FLY_RELATIVE_DIRECTION, 
        //处理请求
        [this](const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const FlyRelative::Goal> goal){
            m_uuid = rclcpp_action::to_string(uuid);
            RCLCPP_INFO(get_logger(), "任务id: %s, 接收的目标航点: forward=%f right=%f up=%f, speed=%f", 
                                    m_uuid.c_str(), 
                                    goal->forward, goal->right, goal->up, goal->speed);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        //处理取消请求
        [this](const std::shared_ptr<GoalHandle> goal_handle){
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "任务id: %s, 目标航点已取消", m_uuid.c_str());
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        //接收并准备执行任务
        [this](const std::shared_ptr<GoalHandle> goal_handle){
            std::thread{std::bind(&FlyRelativeDirectionAction::execute, this, goal_handle)}.detach();
        });
}

void FlyRelativeDirectionAction::execute(
    std::shared_ptr<GoalHandle> goal_handle)
{       
    while(!m_vehicle_odometry_listener.has_received()){};

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FlyRelative::Feedback>();
    auto result = std::make_shared<FlyRelative::Result>();
    auto vehicle_odometry = m_vehicle_odometry_listener.get_msg();
    rclcpp::Rate rate(20);

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

    RCLCPP_INFO(get_logger(), "任务id: %s 开始执行", m_uuid.c_str());
    while(rclcpp::ok()){
        auto current = vehicle_odometry.position;
        float dist = std::sqrt(
            std::pow(current[0] - target[0], 2) +
            std::pow(current[1] - target[1], 2) +
            std::pow(current[2] - target[2], 2));
        feedback->traveled_distance = dist;
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Goal canceled";
            goal_handle->canceled(result);
            return;
        }

        if (dist < 0.2) {
            break;
        }

        common_msgs::msg::TrajectorySetPoint sp{};
        sp.position[0] = target[0];
        sp.position[1] = target[1];
        sp.position[2] = target[2];
        sp.yaw = yaw;

        m_trajectory_setpoint_pub->publish(sp);
        rate.sleep();
    }
    result->success = true;
    result->message = "Reached target";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "任务id: %s 已完成", m_uuid.c_str());
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlyRelativeDirectionAction>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
