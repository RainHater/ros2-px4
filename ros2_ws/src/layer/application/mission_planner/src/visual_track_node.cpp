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
    m_yaml.lift_height = config["lift_height"].as<float>();
    m_yaml.outdoor_flag = config["outdoor_flag"].as<bool>();
    m_yaml.switch_mode = config["switch_mode"].as<bool>();
    m_yaml.track_mode = config["track_mode"].as<int>();

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
    auto& topics = utilities::TopicInfo::getInstance();
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    qos.transient_local();

    m_pub.offboard_mode = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topics.topic_in().PX4_MODE, 10
    );

    m_pub.trajectory_setpoint = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topics.topic_px4_in().PX4_TRAJECTORY_SETPOINT, qos
    );
}

void VisualTrack::init_sub(){
    auto& topics = utilities::TopicInfo::getInstance();
    rclcpp::QoS qos_best_effort(rclcpp::KeepLast(10));
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(1));
    
    qos_best_effort.best_effort();
    qos_reliable.best_effort();
    qos_reliable.transient_local();

    m_sub.offboard_mode.subscribe(
        shared_from_this(), 
        topics.topic_out().PX4_MODE, 10
    );

    m_sub.vehicle_odometry.subscribe(
        shared_from_this(), 
        topics.topic_px4_out().VEHICLE_ODOMETRY, qos_best_effort
    );

    m_sub.local_position.subscribe(
        shared_from_this(),
        topics.topic_px4_out().VEHICLE_LOCAL_POSITION, qos_best_effort
    );

    m_sub.vehicle_attitude.subscribe(
        shared_from_this(), 
        topics.topic_px4_out().VEHICLE_ATTITUDE, qos_reliable
    );

    m_sub.sensor_combined.subscribe(
        shared_from_this(), 
        topics.topic_px4_out().SENSOR_COMBINED, qos_reliable
    );

    m_sub.manual_control_setpoint.subscribe(
        shared_from_this(), 
        topics.topic_px4_out().MANUAL_CONTROL_SETPOINT, qos_reliable
    );

    m_sub.yolo_detections.subscribe(
        shared_from_this(), 
        topics.topic_out().YOLO_DETECTIONS, 10
    );
}

void VisualTrack::task_loop(){
    auto rc_mode = m_interface.rc_signal.get_rc(m_sub.manual_control_setpoint.get_msg());

    if (rc_mode == rc_signal::LAND){
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
                auto vehicle_odometry = m_sub.vehicle_odometry.get_msg();
                std::array<float, 3> cur_pos = vehicle_odometry.position;
                std::array<float, 4> flo_q = vehicle_odometry.q;

                bool arrive = m_interface.movement.change_height(
                        cur_pos, 
                        flo_q, 
                        get_clock()->now(),
                        m_pub_msgs.trajectory_setpoint,
                        m_yaml.lift_height,
                        0.5,
                        m_yaml.outdoor_flag
                    );
                if (arrive){
                    if (vehicle_odometry.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED){
                        RCLCPP_INFO(get_logger(),
                            "当前为 NED 坐标系"
                        );
                    }else if (vehicle_odometry.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD){
                        RCLCPP_INFO(get_logger(),
                            "当前为 FRD 坐标系"
                        );
                    }
                    auto switch_mode = m_yaml.switch_mode;
                    if (switch_mode){
                        m_fly = SWITCH_MODE;
                    }else {
                        m_fly = WAIT;
                    }
                    RCLCPP_INFO(get_logger(), "上升完成!");
                }
            }
            break;
        }
        case SWITCH_MODE:{
            m_interface.mode_control.unlock(
                ARM_ENABLE, VELOCITY, 
                m_sub.offboard_mode.get_msg(), 
                m_pub_msgs.offboard_mode
            );
            if (m_interface.mode_control.wait_busy()){
                m_fly = WAIT;
                RCLCPP_INFO(get_logger(), "切换到速度模式!");
            }
            break;
        }
        case WAIT: {
            auto has_received = m_sub.yolo_detections.has_received();
            if (has_received){
                m_fly = Hover;
                RCLCPP_INFO(get_logger(), "数据有效!");
            }
            break;
        }
        case Hover:{
            auto has_received = m_sub.yolo_detections.has_change();
            auto track_mode = m_yaml.track_mode;
            auto detections = m_sub.yolo_detections.get_msg();
            auto cur_pos = m_sub.vehicle_odometry.get_msg().position;
            auto flo_q = m_sub.vehicle_odometry.get_msg().q;


            m_interface.track.update_last_postition(cur_pos);
            if (has_received){
                if (track_mode == 0){
                    m_interface.track.normal_track(
                        detections,
                        cur_pos,
                        flo_q,
                        m_pub_msgs.trajectory_setpoint
                    );
                }else if (track_mode == 1){
                    m_interface.track.normal_track_v1(
                        detections,
                        cur_pos,
                        flo_q,
                        m_pub_msgs.trajectory_setpoint
                    );
                }else if (track_mode == 2){
                    m_interface.track.normal_track_v2(
                        detections,
                        cur_pos,
                        flo_q,
                        m_pub_msgs.trajectory_setpoint
                    );
                }else if (track_mode == 3){
                    m_interface.track.normal_track_v3(
                        detections,
                        cur_pos,
                        flo_q,
                        m_pub_msgs.trajectory_setpoint
                    );
                }
            }
            break;
        }
        case LAND:{
            std::array<float, 4> flo_q = m_sub.vehicle_odometry.get_msg().q;

            m_interface.movement.land_mode(flo_q, m_pub_msgs.trajectory_setpoint);
            break;
        }
    }

    auto timestamp = get_clock()->now().nanoseconds() / 1000;
    m_pub_msgs.offboard_mode.timestamp = timestamp;
   
    m_pub.offboard_mode->publish(m_pub_msgs.offboard_mode);

    if (m_sub.offboard_mode.get_msg().arm == ARM_ENABLE){
        m_pub_msgs.trajectory_setpoint.timestamp = timestamp;

        m_pub.trajectory_setpoint->publish(m_pub_msgs.trajectory_setpoint);
    }
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
