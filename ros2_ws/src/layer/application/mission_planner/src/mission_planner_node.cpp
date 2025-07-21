#include "mission_planner/mission_planner_node.h"
#include "utilities/topic_name.hpp"
#include "control_interface/gps_nav_api.h"
#include <cmath>
#include <optional>

using std::placeholders::_1;

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner_node"), 
    m_pid_x(0.038f, 0.0000000000000001, 0.0000000000099f),
    m_pid_y(0.038f, 0.0000000000000001, 0.0000000000099f)
{
    RCLCPP_INFO(get_logger(), "Starting mission_planner_node follower node...");
}

void MissionPlanner::initialized(){
    init_publisher();  
    init_subscription();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&MissionPlanner::timer_callback, this));
}

void MissionPlanner::init_publisher(){
    m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_pub::SET_OFFBOARD_MODE, 10);

    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_pub::TRAJECTORY_SETPOINT, 10);

    m_pid_viewer_pub = create_publisher<common_msgs::msg::PidDebug>(
        topic_pub::PID_VIEWER, 10);
}

void MissionPlanner::init_subscription(){
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        topic_sub::PX4_MODE_STATUS, 10, 
        std::bind(&MissionPlanner::px4_mode_status_callback, this, _1)
    );
    
    m_tracking_feedback_listener.subscribe(
        shared_from_this(), 
        topic_sub::TRACKING_FEEDBACK, 10);
} 

void MissionPlanner::timer_callback(){
    auto tracking_feedback = m_tracking_feedback_listener.get_msg();

    if (m_current_task_status == FLY_TO_READY_POSITION){
        NavApi::Instance().send_goal(shared_from_this(), 
        0.000004998, 0.0000600, 2.0, 
        [this](){
            m_current_task_status = FLY_TO_READY_POSITION_AND_LAND;
        });
    }else if (m_current_task_status == FLY_TO_READY_POSITION_AND_LAND){
        NavApi::Instance().send_goal(shared_from_this(), 
        0.000004998, 0.0000600,  0.0, 
        [this](){
            m_current_task_status = FLY_TO_GPS_TARGET;
        });
    }else if (m_current_task_status == FLY_TO_GPS_TARGET){
        NavApi::Instance().send_goal(shared_from_this(), 
        0.0000047, 0.0000009,  8, 
        [this](){
            m_current_task_status = SWITCH_TO_OFFBOARD_VELOCITY_MODE;
        });
    }else if (m_current_task_status == SWITCH_TO_OFFBOARD_VELOCITY_MODE){
        common_msgs::msg::ArmOffboardStatus msgs{};
        msgs.offboard_mode = msgs.VELOCITY;
        m_set_offboard_mode_pub->publish(msgs);
    }else if (m_current_task_status == VISUAL_TRACKING){
        float pixel_threshold = 10.0f;
        common_msgs::msg::TrajectorySetPoint msgs{};
        
        float vx = -m_pid_y.update(tracking_feedback.angle_y);     //正值向后，负值向前
        float vy = m_pid_x.update(tracking_feedback.angle_x);    //正值向左，负值向右
        
        msgs.velocity[0] = vx;  
        msgs.velocity[1] = vy;
        msgs.velocity[2] = (tracking_feedback.pixel_dist < pixel_threshold)?2.0f:0.0f;
        msgs.yawspeed = 0.0f;
        m_trajectory_set_point_pub->publish(msgs);
        common_msgs::msg::PidDebug debug_msg{};
        debug_msg.pixel_dist = tracking_feedback.pixel_dist;
        debug_msg.angle_y = tracking_feedback.angle_y;
        debug_msg.angle_x = tracking_feedback.angle_x;
        debug_msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        m_pid_viewer_pub->publish(debug_msg);
        RCLCPP_INFO(get_logger(), "velocity[0]: %f, velocity[1]: %f, velocity[2]: %f, yawspeed: %f, pixel_dist: %f", 
                    msgs.velocity[0],
                    msgs.velocity[1],
                    msgs.velocity[2],
                    msgs.yawspeed,
                    tracking_feedback.pixel_dist);
    }else if (m_current_task_status == LANDING){
        common_msgs::msg::TrajectorySetPoint msgs{};
        msgs.velocity[0] = 0.0f;
        msgs.velocity[1] = 0.0f;
        msgs.velocity[2] = 2.0f;
        msgs.yawspeed = 0.0f;
        m_trajectory_set_point_pub->publish(msgs);
    }
}

void MissionPlanner::px4_mode_status_callback(
    const common_msgs::msg::ArmOffboardStatus::SharedPtr msg)
{
    if (msg->offboard_mode == common_msgs::msg::ArmOffboardStatus::POSITION){
        if (m_current_task_status==WAIT_FOR_ARM_AND_OFFBOARD){
            m_current_task_status = FLY_TO_READY_POSITION;
        }
    }else if (msg->offboard_mode == common_msgs::msg::ArmOffboardStatus::VELOCITY){
        if (m_current_task_status==SWITCH_TO_OFFBOARD_VELOCITY_MODE){
            m_current_task_status = VISUAL_TRACKING;
        }
    }
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionPlanner>();
    node->initialized();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

