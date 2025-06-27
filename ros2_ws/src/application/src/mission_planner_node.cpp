#include "application/mission_planner_node.h"
#include <rclcpp/logging.hpp>

using std::placeholders::_1;

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner_node") {
    RCLCPP_INFO(get_logger(), "Starting mission_planner_node follower node...");

    init_publisher();
    init_subscription();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MissionPlanner::timer_callback, this));
}

void MissionPlanner::init(){
    m_nav_controller = std::make_shared<NavigationController>(shared_from_this());
}

void MissionPlanner::init_publisher(){
    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TargetGps>(
        "/control/target_gps", 10);
}

void MissionPlanner::init_subscription(){
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10, 
        std::bind(&MissionPlanner::px4_mode_status_callback, this, _1));
} 

void MissionPlanner::timer_callback(){
    if (m_current_task_status == FLY_TO_READY_POSITION){
        m_nav_controller->fly_to(0.000004998, 0.0000600, 2.0, [this](NavigationController::GoalHandleNavigate::WrappedResult result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(get_logger(), "导航成功：%s", result.result->message.c_str());
                m_current_task_status = FLY_TO_READY_POSITION_AND_LAND;
            }
        });
    }else if (m_current_task_status == FLY_TO_READY_POSITION_AND_LAND){
        m_nav_controller->fly_to(0.000004998, 0.0000600,  0.0, [this](NavigationController::GoalHandleNavigate::WrappedResult result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "导航成功：%s", result.result->message.c_str());
            m_current_task_status = FLY_TO_GPS_TARGET;
        }});
    }else if (m_current_task_status == FLY_TO_GPS_TARGET){
        m_nav_controller->fly_to(0.0000047, 0.0000009,  2.0, [this](NavigationController::GoalHandleNavigate::WrappedResult result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "导航成功：%s", result.result->message.c_str());
            m_current_task_status = FLY_TO_GPS_TARGET_AND_LAND;
        }});
    }else if (m_current_task_status == FLY_TO_GPS_TARGET_AND_LAND){
        m_nav_controller->fly_to(0.0000047, 0.0000009,  0.0, [this](NavigationController::GoalHandleNavigate::WrappedResult result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "导航成功：%s", result.result->message.c_str());
            m_current_task_status = FLY_TO_READY_POSITION;
        }});
    }
}

void MissionPlanner::px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg){
    if (msg->offboard_mode == common_msgs::msg::ArmOffboardStatus::POSITION){
        m_current_task_status = FLY_TO_READY_POSITION;
    }
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<MissionPlanner>();
    planner->init();
    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
