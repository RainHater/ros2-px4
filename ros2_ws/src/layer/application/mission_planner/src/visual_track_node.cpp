#include "mission_planner/visual_track_node.h"

#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

using std::placeholders::_1;

VisualTrack::VisualTrack()
    : rclcpp::Node("visual_track")
    , m_dt(10)
    , m_fly(IDLE)
{
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["visual_track"];
    m_yaml.lift_height = config["lift_height"].as<float>();
    m_yaml.track_mode = config["track_mode"].as<int>();
    m_yaml.outdoor_flag = config["outdoor_flag"].as<bool>();
    m_yaml.switch_mode = config["switch_mode"].as<bool>();
    m_yaml.is_loc_pos = config["is_loc_pos"].as<bool>();

    m_drone_data.detect_last_dt = get_ms();

    RCLCPP_INFO(get_logger(), "-----------跟踪配置文件----------");
    RCLCPP_INFO(get_logger(), "起飞高度: %f", m_yaml.lift_height);
    RCLCPP_INFO(get_logger(), "跟踪模式: %d", m_yaml.track_mode);
    RCLCPP_INFO(get_logger(), "是否户外: %d", m_yaml.outdoor_flag);
    RCLCPP_INFO(get_logger(), "是否切换速度模式: %d", m_yaml.switch_mode);
    RCLCPP_INFO(get_logger(), "是否使用loc_pos: %d", m_yaml.is_loc_pos);
    RCLCPP_INFO(get_logger(), "m_dt: %ld", m_dt);
    RCLCPP_INFO(get_logger(), "-----------配置文件结束----------");

    RCLCPP_INFO(get_logger(), "visual_track 节点启动...");
}

void VisualTrack::initialize(){

    initPub();
    initSub();
    
    m_timer = create_wall_timer(
        std::chrono::milliseconds(m_dt),
        std::bind(&VisualTrack::taskLoop, this)
    );
}

void VisualTrack::initPub(){
    auto& topics = utilities::TopicInfo::getInstance();
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    qos.transient_local();

    m_pub.offb_mode = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topics.topic_in().PX4_MODE, 10
    );

    m_pub.traj = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topics.topic_px4_in().PX4_TRAJECTORY_SETPOINT, qos
    );
}

void VisualTrack::initSub(){
    auto& topics = utilities::TopicInfo::getInstance();
    rclcpp::QoS qos_best_effort(rclcpp::KeepLast(10));
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(1));
    
    qos_best_effort.best_effort();
    qos_reliable.best_effort();
    qos_reliable.transient_local();

    m_sub.vehicle_odom = create_subscription<px4_msgs::msg::VehicleOdometry>(
        topics.topic_px4_out().VEHICLE_ODOMETRY,
        qos_best_effort,
        std::bind(&VisualTrack::vehicleOdometryCallback, this, _1)
    );

    m_sub.yolo_dets = create_subscription<identify::msg::YoloDetections>(
        topics.topic_out().YOLO_DETECTIONS,
        10,
        std::bind(&VisualTrack::yoloDetectionsCallback, this, _1)
    ); 

    m_sub.manual_ctrl_sp = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        topics.topic_px4_out().MANUAL_CONTROL_SETPOINT,
        qos_reliable,
        std::bind(&VisualTrack::manualControlSetpointCallback, this, _1)
    ); 

    m_sub.offb_mode = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        topics.topic_out().PX4_MODE,
        10,
        std::bind(&VisualTrack::offboardModeCallback, this, _1)
    ); 

    m_sub.global_pos = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        topics.topic_px4_out().VEHICLE_GLOBAL_POSITION,
        qos_best_effort,
        std::bind(&VisualTrack::globalPosCallback, this, _1)
    );
    
    m_sub.loc_pos = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        topics.topic_px4_out().VEHICLE_LOCAL_POSITION,
        qos_best_effort,
        std::bind(&VisualTrack::locPosCallback, this, _1)
    );
}

void VisualTrack::taskLoop(){
    auto cur_arm = m_drone_data.cur_arm;
    auto cur_offb = m_drone_data.cur_offb;
    auto cur_pos = (m_yaml.is_loc_pos)?m_drone_data.loc_pos:m_drone_data.cur_pos;
    auto flo_q = m_drone_data.flo_q;

    switch(m_fly){
        case IDLE:{
            m_iface.mode_ctrl.unlock(
                ARM_ENABLE, POSITION, 
                cur_arm, cur_offb,
                m_pub_msgs.offb_mode
            );
            if (m_iface.mode_ctrl.waitBusy()){
                m_fly = RISE;
                RCLCPP_INFO(get_logger(), "初始化完成!");
            }
            break;
        }
        case RISE:{
            bool arrive = m_iface.movement.changeHeight(
                    cur_pos, 
                    flo_q, 
                    get_clock()->now(),
                    m_pub_msgs.traj,
                    m_yaml.lift_height,
                    0.5,
                    m_yaml.outdoor_flag
                );
            if (arrive){
                auto cur_coor = m_drone_data.cur_coor;
                if (cur_coor == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED){
                    RCLCPP_INFO(get_logger(),
                        "当前为 NED 坐标系"
                    );
                }else if (cur_coor == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD){
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
                m_drone_data.cal_pos = m_pub_msgs.traj.position;
                RCLCPP_INFO(get_logger(), "上升完成!");
            }
            break;
        }
        case SWITCH_MODE:{
            m_iface.mode_ctrl.unlock(
                ARM_ENABLE, VELOCITY, 
                cur_arm, cur_offb,
                m_pub_msgs.offb_mode
            );
            if (m_iface.mode_ctrl.waitBusy()){
                m_fly = WAIT;
                RCLCPP_INFO(get_logger(), "切换到速度模式!");
            }
            break;
        }
        case WAIT: {
            if (m_drone_data.is_detection_changed){
                m_fly = Hover;
                RCLCPP_INFO(get_logger(), "数据有效!");
            }
            break;
        }
        case Hover:{
            auto track_mode = m_yaml.track_mode;
            auto det_targets = m_drone_data.det_targets;
            auto is_target_valid = m_drone_data.is_target_valid;
            auto dt = m_drone_data.detect_dt;

            m_iface.track.updateLastPostition(cur_pos);
            if (m_drone_data.is_detection_changed){
                m_drone_data.is_detection_changed = false;
                if (track_mode == 0){
                    m_iface.track.normalTrack(
                        is_target_valid,
                        dt,
                        cur_pos,
                        flo_q,
                        det_targets,
                        m_pub_msgs.traj
                    );
                }else if (track_mode == 1){
                    m_iface.track.normalTrack_v1(
                        is_target_valid,
                        dt,
                        cur_pos,
                        flo_q,
                        det_targets,
                        m_pub_msgs.traj
                    );
                }else if (track_mode == 2){
                    m_iface.track.normalTrack_v2(
                        is_target_valid,
                        dt,
                        cur_pos,
                        flo_q,
                        det_targets,
                        m_pub_msgs.traj
                    );
                }else if (track_mode == 3){
                    m_iface.track.normalTrack_v3(
                        is_target_valid,
                        dt,
                        cur_pos,
                        flo_q,
                        det_targets,
                        m_pub_msgs.traj
                    );
                }else if (track_mode == 4){
                    m_iface.track.normalTrack_v4(
                        is_target_valid,
                        dt,
                        cur_pos,
                        m_drone_data.cal_pos,
                        flo_q,
                        det_targets,
                        m_pub_msgs.traj
                    );
                }else if (track_mode == 5){
                    m_iface.track.normalTrack_v5(
                        is_target_valid,
                        dt,
                        cur_pos,
                        flo_q,
                        det_targets,
                        m_pub_msgs.traj
                    );
                }
            }
            if (is_target_valid){
                auto now_ms = m_dt / 1000.0f;
                m_drone_data.cal_pos[0] += m_pub_msgs.traj.velocity[0] * now_ms;
                m_drone_data.cal_pos[1] += m_pub_msgs.traj.velocity[1] * now_ms;
                m_drone_data.cal_pos[2] += m_pub_msgs.traj.velocity[2] * now_ms;
            }
            break;
        }
        case LAND:{
            m_iface.movement.landMode(flo_q, m_pub_msgs.traj);
            break;
        }
    }

    auto timestamp = get_clock()->now().nanoseconds() / 1000;
    m_pub_msgs.offb_mode.timestamp = timestamp;
   
    m_pub.offb_mode->publish(m_pub_msgs.offb_mode);

    if (cur_arm == ARM_ENABLE){
        m_pub_msgs.traj.timestamp = timestamp;
        m_pub.traj->publish(m_pub_msgs.traj);
    }
}

int64_t VisualTrack::get_ms(){
    return get_clock()->now().nanoseconds() / 1000000; 
}

void VisualTrack::vehicleOdometryCallback(
    const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg
){
    m_drone_data.cur_pos = msg->position;
    m_drone_data.flo_q = msg->q;
    m_drone_data.cur_coor = msg->pose_frame;
}

void VisualTrack::yoloDetectionsCallback(
    const std::shared_ptr<identify::msg::YoloDetections> msg
){  
    auto now_ms = get_ms();
    m_drone_data.detect_dt = now_ms - m_drone_data.detect_last_dt;
    m_drone_data.detect_last_dt = now_ms;
    m_drone_data.det_targets = msg->detections;
    m_drone_data.is_target_valid = msg->detect_flag;
    m_drone_data.is_detection_changed = true;

    // RCLCPP_INFO(get_logger(), "detect_dt: %ld", m_drone_data.detect_dt);
}

void VisualTrack::manualControlSetpointCallback(
    const std::shared_ptr<px4_msgs::msg::ManualControlSetpoint> msg
){
    if (msg->aux1 < 0){
        m_fly = LAND;
    }
}

void VisualTrack::offboardModeCallback(
    const std::shared_ptr<common_msgs::msg::ArmOffboardStatus> msg
){
    m_drone_data.cur_arm = msg->arm;
    m_drone_data.cur_offb = msg->offboard;
}

void VisualTrack::globalPosCallback(
    const std::shared_ptr<px4_msgs::msg::VehicleGlobalPosition> msg
){
    m_drone_data.cur_gps.lat = msg->lat;
    m_drone_data.cur_gps.lon = msg->lon;
    m_drone_data.cur_gps.alt = msg->alt;
}

void VisualTrack::locPosCallback(
    const std::shared_ptr<px4_msgs::msg::VehicleLocalPosition> msg
){
    if (msg->xy_valid){
        m_drone_data.loc_pos[0] = msg->x;
        m_drone_data.loc_pos[1] = msg->y;
    }
    if (msg->z_valid){
        m_drone_data.loc_pos[2] = msg->z;
    }
    
    // if (msg->xy_valid && msg->z_valid){
    //     RCLCPP_INFO(
    //         get_logger(), 
    //         "loc_p[0]: %f, loc_p[1]: %f, loc_p[2]: %f",
    //         m_drone_data.loc_pos[0],
    //         m_drone_data.loc_pos[1],
    //         m_drone_data.loc_pos[2]
    //     );
    // }
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
