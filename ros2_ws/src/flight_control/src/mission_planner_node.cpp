#include "flight_control/mission_planner_node.h"

using std::placeholders::_1;
using std::placeholders::_2;

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner_node") {
    RCLCPP_INFO(get_logger(), "Starting mission_planner_node follower node...");

    m_nav_cllient = rclcpp_action::create_client<NavigateToGPS>(this, "/control/navigate_to_gps");

    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TargetGps>(
        "/control/target_gps", 10);
    
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10, 
        std::bind(&MissionPlanner::px4_mode_status_callback, this, _1));
    
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MissionPlanner::timer_callback, this));
}

void MissionPlanner::timer_callback(){
    switch(m_current_task_status){
        case FLY_TO_READY_POSITION:{
            nav_sends_goal();
        }
        break;
        case FLY_TO_GPS_TARGET:{
            common_msgs::msg::TargetGps target_gps;
            target_gps.lat = 0.0000047;
            target_gps.lon = 0.0000009;
            target_gps.alt = 2.0;
            m_trajectory_setpoint_pub->publish(target_gps);
        }
        break;
    }
}

void MissionPlanner::px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg){
    if (msg->offboard_mode == common_msgs::msg::ArmOffboardStatus::POSITION){
        m_current_task_status = FLY_TO_READY_POSITION;
    }
}

void MissionPlanner::nav_goal_response_callback(std::shared_ptr<GoalHandleNavigate> future){
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result...");
    }
}

void MissionPlanner::nav_feedback_callback(
        GoalHandleNavigate::SharedPtr, 
        const std::shared_ptr<const NavigateToGPS::Feedback> feedback){

    // RCLCPP_INFO(get_logger(), "Feedback: lat=%f lon=%f alt=%f remaining=%.2f",
    //                 feedback->current_latitude,
    //                 feedback->current_longitude,
    //                 feedback->current_altitude,
    //                 feedback->distance_remaining);
}

void MissionPlanner::nav_result_callback(const GoalHandleNavigate::WrappedResult &result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Result: success=%d message=%s", result.result->success, result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
    }
}

void MissionPlanner::nav_sends_goal(){
    if(m_action_state)
        return;
    m_action_state = true;

    if (!m_nav_cllient->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Action server not available.");
        return;
    }

    auto goal_msg = NavigateToGPS::Goal();
    goal_msg.lat = 0.000004998;
    goal_msg.lon = 0.0000600;
    goal_msg.alt = 2.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal: lat=%lf lon=%lf alt=%lf", 
        goal_msg.lat, 
        goal_msg.lon, 
        goal_msg.alt);
    
    rclcpp_action::Client<NavigateToGPS>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&MissionPlanner::nav_goal_response_callback, this, _1);
    options.feedback_callback = std::bind(&MissionPlanner::nav_feedback_callback, this, _1, _2);
    options.result_callback = std::bind(&MissionPlanner::nav_result_callback, this, _1);

    m_nav_cllient->async_send_goal(goal_msg, options);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanner>());

    rclcpp::shutdown();
    return 0;
}
