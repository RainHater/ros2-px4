#include "application/mission_planner_node.h"
#include <cmath>
#include <optional>
#include "control/controller_api.h"
#include "utilities/util_topic.hpp"

using std::placeholders::_1;

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner_node"), 
    m_pid_x(0.038f, 0.0000000000000001, 0.0000000000099f),
    m_pid_y(0.038f, 0.0000000000000001, 0.0000000000099f) {
    RCLCPP_INFO(get_logger(), "Starting mission_planner_node follower node...");

    init_publisher();  
    init_subscription();
    init_client();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MissionPlanner::timer_callback, this));
}

void MissionPlanner::init_publisher(){
    m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        "/control/set_offboard_mode", 10);
    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
    m_pid_viewer_pub = create_publisher<common_msgs::msg::PidDebug>(
        "/debug/pid_viewer", 10);
}

void MissionPlanner::init_subscription(){
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10, 
        std::bind(&MissionPlanner::px4_mode_status_callback, this, _1));
    
    m_tracking_feedback_sub = utils::make_simple_subscription<common_msgs::msg::TrackingFeedback>(
        "/vision_pipeline/tracking_feedback", 10, 
        this, 
        m_tracking_feedback);
} 

void MissionPlanner::init_client(){
    m_nav_client = rclcpp_action::create_client<NavigateToGPS>(
        this, "/control/navigate_to_gps");
}

void MissionPlanner::timer_callback(){
    if (m_current_task_status == FLY_TO_READY_POSITION){
        ControllerApi::NavApi::Instance().send_goal(shared_from_this(), 
        m_nav_client, 0.000004998, 0.0000600, 2.0, 
        [this](){
            m_current_task_status = FLY_TO_READY_POSITION_AND_LAND;
        });
    }else if (m_current_task_status == FLY_TO_READY_POSITION_AND_LAND){
        ControllerApi::NavApi::Instance().send_goal(shared_from_this(), 
        m_nav_client, 0.000004998, 0.0000600,  0.0, 
        [this](){
            m_current_task_status = FLY_TO_GPS_TARGET;
        });
    }else if (m_current_task_status == FLY_TO_GPS_TARGET){
        ControllerApi::NavApi::Instance().send_goal(shared_from_this(), 
        m_nav_client, 0.0000047, 0.0000009,  5, 
        [this](){
            m_current_task_status = SWITCH_TO_OFFBOARD_VELOCITY_MODE;
        });
    }else if (m_current_task_status == SWITCH_TO_OFFBOARD_VELOCITY_MODE){
        common_msgs::msg::ArmOffboardStatus msgs{};
        msgs.offboard_mode = msgs.VELOCITY;
        m_set_offboard_mode_pub->publish(msgs);
    }else if (m_current_task_status == VELOCITY_OFFBOARD_READY){
        float pixel_threshold = 5.0f;
        common_msgs::msg::TrajectorySetPoint msgs{};
        
        if (m_tracking_feedback.pixel_dist < pixel_threshold) {
            m_pid_y.reset();
            m_pid_x.reset();
            msgs.velocity[0] = 0.0f;
            msgs.velocity[1] = 0.0f;
            msgs.velocity[2] = 0.5f;
            msgs.yawspeed = 0.0f;
        } else {
            float vx = m_pid_y.update(m_tracking_feedback.angle_y);     //正值向后，负值向前
            float vy = -m_pid_x.update(m_tracking_feedback.angle_x);    //正值向左，负值向右
            
            msgs.velocity[0] = vx;  
            msgs.velocity[1] = vy;
            msgs.velocity[2] = 0.0f;
            msgs.yawspeed = 0.0f;
        }
        m_trajectory_set_point_pub->publish(msgs);
        common_msgs::msg::PidDebug debug_msg{};
        debug_msg.pixel_dist = m_tracking_feedback.pixel_dist;
        debug_msg.angle_y = m_tracking_feedback.angle_y;
        debug_msg.angle_x = m_tracking_feedback.angle_x;
        debug_msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        m_pid_viewer_pub->publish(debug_msg);
        // RCLCPP_INFO(get_logger(), "velocity[0]: %f, velocity[1]: %f, velocity[2]: %f, yawspeed: %f, pixel_dist: %f", 
        //             msgs.velocity[0],
        //             msgs.velocity[1],
        //             msgs.velocity[2],
        //             msgs.yawspeed,
        //             m_tracking_feedback.pixel_dist);
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

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanner>());
    rclcpp::shutdown();
    return 0;
}
