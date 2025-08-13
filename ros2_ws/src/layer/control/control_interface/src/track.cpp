#include "control_interface/track.h"

#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

Track::Track()
 : m_log(rclcpp::get_logger("跟踪(track.cpp)"))
{   
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["track"];
    m_yaml.deviation_kp = config["deviation_kp"].as<float>();
    m_yaml.deviation_ki = config["deviation_ki"].as<float>();
    m_yaml.deviation_kd = config["deviation_kd"].as<float>();

    m_pid.deviation.initialize(m_yaml.deviation_kp, m_yaml.deviation_ki, m_yaml.deviation_kd, false, 180.0f); 
    // m_pid.yawspeed.initialize(0.0015, 0.00001, 0.0f, false); 
}

void Track::normal_track(track::NormalTrack& normal_info) {
    auto& detection = normal_info.detections.detections[0];
    auto& sub_pose = normal_info.sub_pose;
    auto& sub_attitude = normal_info.sub_attitude;
    auto* pub_tra = normal_info.pub_tra;
    
    float setpoint = detection.cx;
    float measured_value = detection.image_width / 2.0f;

    float output = m_pid.deviation.compute(setpoint, measured_value);
    
    pub_tra->yawspeed = output;

    RCLCPP_INFO(m_log, 
        "类别: %s, "
        "cx: %d, cy: %d, yawspeed: %f",
        detection.target_name.c_str(),
        detection.cx, detection.cy,
        pub_tra->yawspeed
    );
}
