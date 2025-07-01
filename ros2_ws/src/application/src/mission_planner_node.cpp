#include "application/mission_planner_node.h"
#include <common_msgs/msg/detail/tracking_feedback__struct.hpp>
#include <optional>
#include <rclcpp/logging.hpp>
#include "control/controller_api.h"
#include "utilities/util_topic.hpp"

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
    m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        "/control/set_offboard_mode", 10);
    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
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
        m_nav_client, 0.0000047, 0.0000009,  2.0, 
        [this](){
            m_current_task_status = SWITCH_TO_OFFBOARD_VELOCITY_MODE;
        });
    }else if (m_current_task_status == SWITCH_TO_OFFBOARD_VELOCITY_MODE){
        common_msgs::msg::ArmOffboardStatus msgs{};
        msgs.offboard_mode = msgs.VELOCITY;
        m_set_offboard_mode_pub->publish(msgs);
    }else if (m_current_task_status == VELOCITY_OFFBOARD_READY){
        float kp_xy = 0.01f; 
        float kp_z =  0.01f;
        float kp_yaw = 0.05f;
        float pixel_threshold = 5.0f;
        common_msgs::msg::TrajectorySetPoint msgs{};
        msgs.velocity[0] = 0;
        msgs.velocity[1] = 0;
        msgs.velocity[2] = 0;
        msgs.yawspeed = 0;
        
        // msgs.velocity[0] = 0.0f;  // 前后控制可通过别的逻辑控制（或 angle_y）
        // msgs.velocity[1] = -kp_xy * m_tracking_feedback.angle_x;  // 横向矫正（右偏为正，速度向左）
        // msgs.velocity[2] = -kp_z  * m_tracking_feedback.angle_y;  // 垂直矫正（下偏为正，速度向上）
        // if (m_tracking_feedback.pixel_dist < pixel_threshold) {
        //     msgs.yawspeed = 0.0f;
        // } else {
        //     msgs.yawspeed = -kp_yaw * m_tracking_feedback.angle;
        // }
        m_trajectory_set_point_pub->publish(msgs);
        RCLCPP_INFO(get_logger(), "pixel_dist: %.f, angle_x: %.f, angle_y: %.f, angle: %.f", 
                    m_tracking_feedback.pixel_dist,
                    m_tracking_feedback.angle_x,
                    m_tracking_feedback.angle_y,
                    m_tracking_feedback.angle);
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
