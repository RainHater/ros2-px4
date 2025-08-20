#include "control_interface/track_2.h"

#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

Track::Track()
 : m_log(rclcpp::get_logger("跟踪(track.cpp)"))
{   
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["track"];
    pos_yaml.deviation_kp = config["pos_kp"].as<float>();
    pos_yaml.deviation_ki = config["pos_ki"].as<float>();
    pos_yaml.deviation_kd = config["pos_kd"].as<float>();

    speed_yaml.deviation_kp = config["speed_kp"].as<float>();
    speed_yaml.deviation_ki = config["speed_ki"].as<float>();
    speed_yaml.deviation_kd = config["speed_kd"].as<float>();

    m_camera.width = 1920;
    m_camera.height = 1080;
    // m_camera.hfov = M_PI * (77.0 / 180.0);

    speed_pid.deviation.initialize(speed_yaml.deviation_kp, speed_yaml.deviation_ki, speed_yaml.deviation_kd, false, 0.785f, 0.785f); 
    pos_pid.deviation.initialize(pos_yaml.deviation_kp, pos_yaml.deviation_ki, pos_yaml.deviation_kd, false)
}

void Track::normal_track(track::NormalTrack& normal_info) {

    auto& cur_ns = normal_info.detections.stamp.nanosec;
    auto& detect_flag = normal_info.detections.detect_flag;
    // auto& sub_pose = normal_info.sub_pose;
    // auto& sub_attitude = normal_info.sub_attitude;
    // auto& sensor_combined = normal_info.sensor_combined;
    auto& pub_tra = *normal_info.pub_tra;

    float cur_time =  cur_ns / 1e6f;
    float dt  = cur_time - m_pid.last_time;
    m_pid.last_time = cur_time;

    float cx = 0.0;
    if (detect_flag){
        auto& detection = normal_info.detections.detections[0];
        cx = detection.cx;
    }else {
        cx = 960.0f;
    }

    float pixel_offset = cx - m_camera.width / 2.0;
    // float angle_offset = (pixel_offset / (m_camera.width / 2.0)) * (m_camera.hfov / 2.0);
    float angle_offset = pixel_offset;
    float setpoint = angle_offset * M_PI / 180.0f;


    float output = m_pid.deviation.compute(setpoint, dt);
    
    pub_tra.yaw = NAN;
    pub_tra.yawspeed = output;

    RCLCPP_INFO(m_log, 
        "cx: %f, setpoint: %f, yawspeed: %f",
        cx, setpoint, 
        pub_tra.yawspeed
    );
}
