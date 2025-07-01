#include "application/mission_planner_node.h"
#include <common_msgs/msg/detail/trajectory_set_point__struct.hpp>
#include <optional>
#include <rclcpp/logging.hpp>

using std::placeholders::_1;

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner_node") {
    RCLCPP_INFO(get_logger(), "Starting mission_planner_node follower node...");

    init_publisher();
    init_subscription();
    init_client();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MissionPlanner::timer_callback, this));
}

void MissionPlanner::init_publisher(){
    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TargetGps>(
        "/control/target_gps", 10);
    m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        "/control/set_offboard_mode", 10);
    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
}

void MissionPlanner::init_subscription(){
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10, 
        std::bind(&MissionPlanner::px4_mode_status_callback, this, _1));
} 

void MissionPlanner::init_client(){
    m_nav_client = rclcpp_action::create_client<NavigateToGPS>(
        this, "/control/navigate_to_gps");
}

void MissionPlanner::timer_callback(){
    if (m_current_task_status == FLY_TO_READY_POSITION){
        send_goal(0.000004998, 0.0000600, 2.0, [this](){
            RCLCPP_INFO(get_logger(), "测试");
            m_current_task_status = FLY_TO_READY_POSITION_AND_LAND;
        });
    }else if (m_current_task_status == FLY_TO_READY_POSITION_AND_LAND){
        send_goal(0.000004998, 0.0000600,  0.0, [this](){
            m_current_task_status = FLY_TO_GPS_TARGET;
        });
    }else if (m_current_task_status == FLY_TO_GPS_TARGET){
        send_goal(0.0000047, 0.0000009,  2.0, [this](){
            m_current_task_status = SWITCH_TO_OFFBOARD_VELOCITY_MODE;
        });
    }else if (m_current_task_status == SWITCH_TO_OFFBOARD_VELOCITY_MODE){
        common_msgs::msg::ArmOffboardStatus msgs{};
        msgs.offboard_mode = msgs.VELOCITY;
        m_set_offboard_mode_pub->publish(msgs);
    }else if (m_current_task_status == VELOCITY_OFFBOARD_READY){
        RCLCPP_INFO(get_logger(), "开始控制");
        common_msgs::msg::TrajectorySetPoint msgs{};
        msgs.velocity[0] = 0;
        msgs.velocity[1] = 0;
        msgs.velocity[2] = -1;
        msgs.yawspeed = 0.5;
        m_trajectory_set_point_pub->publish(msgs);
    }
}

void MissionPlanner::px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg){
    if (msg->offboard_mode == common_msgs::msg::ArmOffboardStatus::POSITION){
        if (m_current_task_status==WAIT_FOR_ARM_AND_OFFBOARD){
            m_current_task_status = FLY_TO_READY_POSITION;
        }
    }else if (msg->offboard_mode == common_msgs::msg::ArmOffboardStatus::VELOCITY){
        if (m_current_task_status==SWITCH_TO_OFFBOARD_VELOCITY_MODE){
            m_current_task_status = VELOCITY_OFFBOARD_READY;
        }
    }
}

void MissionPlanner::send_goal(double lat, double lon, double alt, std::function<void()> succeeded_callback){
    if (m_nav_is_busy)
        return;

    m_nav_is_busy = true;
    if (!m_nav_client->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Action server not available");
        return;
    }
    NavigateToGPS::Goal goal;
    goal.lat = lat;
    goal.lon = lon;
    goal.alt = alt;

    RCLCPP_INFO(get_logger(), "Sending goal: lat=%f lon=%f alt=%f", lat, lon, alt);

    rclcpp_action::Client<NavigateToGPS>::SendGoalOptions options;
    options.goal_response_callback = [this](auto goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected");
        } else {
            RCLCPP_INFO(get_logger(), "Goal accepted by server");
        }
    };
    options.feedback_callback = [](auto, auto) {};
    options.result_callback = [this, succeeded_callback](const GoalHandle::WrappedResult &result){
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (succeeded_callback)
                succeeded_callback();
            RCLCPP_INFO(get_logger(), "导航成功: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "导航任务被中止");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "导航任务被取消");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "未知导航结果状态");
            break;
        }
        m_nav_is_busy = false;
    };
    m_nav_client->async_send_goal(goal, options);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanner>());
    rclcpp::shutdown();
    return 0;
}
