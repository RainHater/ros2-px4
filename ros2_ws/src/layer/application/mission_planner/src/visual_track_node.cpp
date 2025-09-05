#include "mission_planner/visual_track_node.h"

#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

VisualTrack::VisualTrack()
    : rclcpp::Node("visual_track")
{       
    m_fly = IDLE;

    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["visual_track"];
    m_yaml.visual_track = config["lift_height"].as<float>();
    m_yaml.outdoor_flag = config["outdoor_flag"].as<bool>();

    RCLCPP_INFO(get_logger(), "visual_track 节点启动...");
}

void VisualTrack::initialize(){

    init_pub();
    init_sub();
    
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VisualTrack::task_loop, this)
    );
}

void VisualTrack::init_pub(){
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    qos.transient_local();

    m_pub.offboard_mode = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_in::PX4_MODE, 10
    );

    m_pub.trajectory_setpoint = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_px4_in::PX4_TRAJECTORY_SETPOINT, qos
    );
}

void VisualTrack::init_sub(){
    rclcpp::QoS qos_best_effort(rclcpp::KeepLast(10));
    qos_best_effort.best_effort();
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(1));
    qos_reliable.best_effort();
    qos_reliable.transient_local();

    m_sub.offboard_mode.subscribe(
        shared_from_this(), 
        topic_out::PX4_MODE, 10
    );

    m_sub.vehicle_odometry.subscribe(
        shared_from_this(), 
        topic_px4_out::VEHICLE_ODOMETRY, qos_best_effort
    );

    m_sub.local_position.subscribe(
        shared_from_this(),
        topic_px4_out::VEHICLE_LOCAL_POSITION, qos_best_effort
    );

    m_sub.vehicle_attitude.subscribe(
        shared_from_this(), 
        topic_px4_out::VEHICLE_ATTITUDE, qos_reliable
    );

    m_sub.sensor_combined.subscribe(
        shared_from_this(), 
        topic_px4_out::SENSOR_COMBINED, qos_reliable
    );

    m_sub.manual_control_setpoint.subscribe(
        shared_from_this(), 
        topic_px4_out::MANUAL_CONTROL_SETPOINT, qos_reliable
    );

    m_sub.yolo_detections.subscribe(
        shared_from_this(), 
        topic_out::YOLO_DETECTIONS, 10
    );
}

void VisualTrack::task_loop(){
    auto rc_mode = m_interface.rc_signal.get_rc(m_sub.manual_control_setpoint.get_msg());

    if (rc_mode == rc_signal::LAND && m_fly != END){
        m_fly = LAND;
    }

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
            auto msg_arrive = m_sub.vehicle_odometry.has_change();
            if (msg_arrive){
                movement::JustmoveInfo justmove_info;
                justmove_info.sub_pose = m_sub.vehicle_odometry.get_msg();
                justmove_info.pub_pose = &m_pub_msgs.trajectory_setpoint;
                justmove_info.instant_time = get_clock()->now();
                justmove_info.v = 0.1;
                bool arrive = m_interface.movement.change_height(
                        justmove_info, 
                        m_yaml.visual_track, 
                        m_yaml.outdoor_flag
                    );
                if (arrive){
                    m_fly = Hover;
                    RCLCPP_INFO(get_logger(), "上升完成!");
                }
            }
            break;
        }
        case Hover:{
            auto has_received = m_sub.yolo_detections.has_change();
            if (has_received){
                track::NormalTrack normal_track;
                normal_track.detections = m_sub.yolo_detections.get_msg();
                normal_track.sub_pose = m_sub.vehicle_odometry.get_msg();
                normal_track.sub_attitude = m_sub.vehicle_attitude.get_msg();
                normal_track.sensor_combined = m_sub.sensor_combined.get_msg();
                normal_track.pub_tra = &m_pub_msgs.trajectory_setpoint;
                m_interface.track.normal_track(normal_track);
            }
            break;
        }
        case LAND:{
            movement::LandModeInfo land_mode_info;
            land_mode_info.mode_control = m_interface.mode_control;
            land_mode_info.pub_px4_mode = &m_pub_msgs.offboard_mode;
            land_mode_info.pub_pose = &m_pub_msgs.trajectory_setpoint;
            land_mode_info.sub_px4_mode = m_sub.offboard_mode.get_msg();
            land_mode_info.sub_pose = m_sub.vehicle_odometry.get_msg();
            land_mode_info.local_position = m_sub.local_position;
            land_mode_info.instant_time = get_clock()->now();

            bool finish = m_interface.movement.land_mode(land_mode_info, 0.3);
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

void VisualTrack::calculate_yaw(
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
    auto node = std::make_shared<VisualTrack>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
