#include "control_interface/track.h"

#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

Track::Track()
 : m_log(rclcpp::get_logger("跟踪(track.cpp)"))
{   
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["track"];
    m_yaml.cx_kp = config["cx_kp"].as<float>();
    m_yaml.cx_ki = config["cx_ki"].as<float>();
    m_yaml.cx_kd = config["cx_kd"].as<float>();
    m_yaml.cy_kp = config["cy_kp"].as<float>();
    m_yaml.cy_ki = config["cy_ki"].as<float>();
    m_yaml.cy_kd = config["cy_kd"].as<float>();

    m_camera.width = 1920;
    m_camera.height = 1080;

    m_pid.cx.initialize(m_yaml.cx_kp, m_yaml.cx_ki, m_yaml.cx_kd, false, 1.5708f, 1.5708f); 
    m_pid.cy.initialize(m_yaml.cy_kp, m_yaml.cy_ki, m_yaml.cy_kd, false, 0.5f, 0.5f); 
}

void Track::normal_track(track::NormalTrack& normal_info) {

    auto& cur_ns = normal_info.detections.stamp.nanosec;
    auto& detect_flag = normal_info.detections.detect_flag;
    auto& sub_pose = normal_info.sub_pose;
    // auto& sub_attitude = normal_info.sub_attitude;
    // auto& sensor_combined = normal_info.sensor_combined;
    auto& pub_tra = *normal_info.pub_tra;

    float cur_time =  cur_ns / 1e6f;
    float dt  = cur_time - m_pid.last_time;
    m_pid.last_time = cur_time;

    float cx = 0.0f;
    float cy = 0.0f;
    if (detect_flag){
        auto& detection = normal_info.detections.detections[0];
        cx = detection.cx;
        cy = detection.cy;
    }else {
        cx = 960.0f;
        cy = 540.0f;
    }

    float cx_error = cx - m_camera.width / 2.0;
    float cy_error = cy - m_camera.height / 2.0;


    float cx_output = m_pid.cx.compute(cx_error, dt);
    float cy_output = m_pid.cx.compute(cy_error, dt);
    
    pub_tra.yaw = NAN;
    pub_tra.position[0] = sub_pose.position[0];
    pub_tra.position[1] = sub_pose.position[1];
    pub_tra.position[2] = sub_pose.position[2];
    pub_tra.velocity[0] = 0.0f;
    pub_tra.velocity[1] = 0.0f;
    pub_tra.velocity[2] = cy_output;
    pub_tra.yawspeed = cx_output;

    if (!detect_flag)
        return;
    RCLCPP_INFO(m_log, 
        "cx: %f, cx_error: %f, cx_out: %f, "
        "cy: %f, cy_error: %f, cy_out: %f",
        cx, cx_error, 
        pub_tra.yawspeed,
        cy, cy_error,
        pub_tra.velocity[2]
    );
}
