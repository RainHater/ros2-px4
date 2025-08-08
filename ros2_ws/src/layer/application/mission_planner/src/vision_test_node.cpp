#include "mission_planner/vision_test_node.h"
#include <rclcpp/logging.hpp>
#include <utilities/tf2_tool.hpp>
#include <utilities/topic_name.hpp>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

VisionTestNode::VisionTestNode()
    : rclcpp::Node("vision_test_node")
{   
    m_fly = IDLE;
    RCLCPP_INFO(get_logger(), "vision_test_node 节点启动...");
}

void VisionTestNode::initialize(){

    init_pub();
    init_sub();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VisionTestNode::task_loop, this)
    );
}

void VisionTestNode::init_pub(){
    m_pub.offboard_mode = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_in::PX4_MODE, 10
    );

    m_pub.trajectory_setpoint = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_px4_in::PX4_TRAJECTORY_SETPOINT, 10
    );
}

void VisionTestNode::init_sub(){
    m_sub.offboard_mode.subscribe(
        shared_from_this(), 
        topic_out::PX4_MODE, 10
    );

    m_sub.vehicle_odometry.subscribe(
        shared_from_this(), 
        topic_px4_out::VEHICLE_ODOMETRY, 10
    );

    m_sub.local_position.subscribe(
        shared_from_this(),
        topic_px4_out::VEHICLE_LOCAL_POSITION, 10
    );

    m_sub.yolo_detections.subscribe(
        shared_from_this(), 
        topic_out::YOLO_DETECTIONS, 10
    );
}

void VisionTestNode::task_loop(){
    switch(m_fly){
        case IDLE:{
            m_interface.mode_control.unlock(
                ARM_ENABLE, POSITION, 
                m_sub.offboard_mode.get_msg(), 
                m_pub_msgs.offboard_mode
            );
            if (m_interface.mode_control.wait_busy()){
                m_fly = RISE;
                RCLCPP_INFO(get_logger(), "初始化完成!");
            }
            break;
        }
        case RISE:{
            bool arrive = m_interface.movement.change_height(
                m_sub.vehicle_odometry.get_msg(),
                m_pub_msgs.trajectory_setpoint,
                get_clock()->now(),
                0.5,
                0.15
            );
            if (arrive){
                m_fly = Hover;
                RCLCPP_INFO(get_logger(), "上升完成!");
            }
            break;
        }

        case Hover:{
            auto has_received = m_sub.yolo_detections.has_received();
            if (has_received){
                auto detection = m_sub.yolo_detections.get_msg().detections[0];
                RCLCPP_INFO(
                    get_logger(), 
                    "类别: %s, 置信度: %f"
                    "cx: %d, cy: %d",
                    detection.target_name.c_str(),
                    detection.confidence,
                    detection.cx, detection.cy
                );
                calculate_yaw(
                    detection.cx, 
                    detection.image_width, 100, 
                    m_sub.vehicle_odometry.get_msg(), 
                    m_pub_msgs.trajectory_setpoint
                );
            }
            RCLCPP_INFO(get_logger(), "has_received: %d", has_received);
            break;
        }
        case LAND:{
            bool finish = m_interface.movement.land_mode(
                0.3,
                m_interface.mode_control,
                get_clock()->now(),
                m_sub.offboard_mode.get_msg(),
                m_sub.vehicle_odometry.get_msg(),
                m_pub_msgs.trajectory_setpoint,
                m_pub_msgs.offboard_mode,
                m_sub.local_position
            );
            if (finish){
                m_fly = END;
                RCLCPP_INFO(get_logger(), "降落完成!");
            }
            break;
        }
        case END:{
            m_interface.mode_control.locked(
                m_sub.offboard_mode.get_msg(), 
                m_pub_msgs.offboard_mode
            );
            break;
        }
    }

    auto timestamp = get_clock()->now().nanoseconds() / 1000;

    m_pub_msgs.trajectory_setpoint.timestamp = timestamp;

    m_pub.offboard_mode->publish(m_pub_msgs.offboard_mode);
    if (m_sub.offboard_mode.get_msg().arm == ARM_ENABLE){
        m_pub.trajectory_setpoint->publish(m_pub_msgs.trajectory_setpoint);
    }
}

void VisionTestNode::calculate_yaw(
    int cx, 
    int image_width, 
    float fov_deg,
    px4_msgs::msg::VehicleOdometry current,
    px4_msgs::msg::TrajectorySetpoint &pose)
{
    tf2_tool::EulerAngles angles;
    tf2_tool::get_euler_angles(current, angles);
    float dx = cx - image_width / 2.0f;
    float half_width = image_width / 2.0f;

    float angle_offset_rad = (dx / half_width) * (fov_deg * M_PI / 180.0f / 2.0f);
    float target_yaw_rad = angles.yaw + angle_offset_rad;

    pose.yaw = target_yaw_rad;
    RCLCPP_INFO(get_logger(), 
        "当前yaw: %f, 转动yaw: %f, 目标yaw: %f",
        angles.yaw, angle_offset_rad, target_yaw_rad
    );
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionTestNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
