#include "control_interface/track.h"

#include <cmath>
#include <utilities/tf2_tool.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

Track::Track()
 :  m_log(rclcpp::get_logger("跟踪(track.cpp)")),
    m_logger("/home/sunrise/ros2_px4/logs/track")
{   
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["track"];
    m_yaml.yaw.kp = config["yaw_kp"].as<float>();
    m_yaml.yaw.ki = config["yaw_ki"].as<float>();
    m_yaml.yaw.kd = config["yaw_kd"].as<float>();
    m_yaml.ud.kp = config["ud_kp"].as<float>();
    m_yaml.ud.ki = config["ud_ki"].as<float>();
    m_yaml.ud.kd = config["ud_kd"].as<float>();
    m_yaml.fb.kp = config["fb_kp"].as<float>();
    m_yaml.fb.ki = config["fb_ki"].as<float>();
    m_yaml.fb.kd = config["fb_kd"].as<float>();
    m_yaml.area_th = config["area_th"].as<float>();

    m_camera.width = 1920;
    m_camera.height = 1080;

    m_normal_track.thre_area = m_camera.width*m_camera.height*m_yaml.area_th;

    m_pid.yaw.initialize(m_yaml.yaw.kp, m_yaml.yaw.ki, m_yaml.yaw.kd, false, 1.5708f, 1.5708f); 
    m_pid.ud.initialize(m_yaml.ud.kp, m_yaml.ud.ki, m_yaml.ud.kd, false, 0.5f, 0.5f); 
    m_pid.fb.initialize(m_yaml.fb.kp, m_yaml.fb.ki, m_yaml.fb.kd, false, 0.4f, 0.4f);
}

void Track::normal_track(track::NormalTrack& normal_info) {

    auto& cur_ns = normal_info.detections.stamp.nanosec;
    auto& detect_flag = normal_info.detections.detect_flag;
    auto& sub_pose = normal_info.sub_pose;
    // auto& sub_attitude = normal_info.sub_attitude;
    // auto& sensor_combined = normal_info.sensor_combined;
    auto& pub_tra = *normal_info.pub_tra;

    // double cur_time =  cur_ns / 1e6f;
    float dt  = 100;
    // m_pid.last_time = cur_time;

    tf2_tool::EulerAngles angles;
    tf2_tool::get_euler_angles(sub_pose, angles);

    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = angles.yaw;

    if (detect_flag){
        auto& detection = normal_info.detections.detections[0];
        cx = detection.cx;
        cy = detection.cy;
        area = abs((detection.x_max - detection.x_min) * (detection.y_max - detection.y_min));  
    }else {
        cx = 960.0f;
        cy = 540.0f;
        area = m_normal_track.thre_area;
    }

    float cx_error = cx - m_camera.width / 2.0;
    float cy_error = cy - m_camera.height / 2.0;
    float area_error = m_normal_track.thre_area - area;

    float yaw_output = m_pid.yaw.compute(cx_error, dt);
    float ud_output = m_pid.ud.compute(cy_error, dt);
    float fb_output = m_pid.fb.compute(area_error, dt);

    pub_tra.yaw = NAN;
    pub_tra.velocity[0] = cosf(yaw) * fb_output;
    pub_tra.velocity[1] = sinf(yaw) * fb_output;
    pub_tra.velocity[2] = ud_output;
    pub_tra.yawspeed = yaw_output;

    if (!m_pid.last_pos_init){
        m_pid.last_postition = sub_pose.position;
        m_pid.last_pos_init = true;
    }

    if (detect_flag){
        m_pid.last_postition = sub_pose.position;
        pub_tra.position = m_pid.last_postition;
        m_logger.info(m_log, 
            "dt: %f, "
            "cx: %f, cx_error: %f, yaw_out: %f, "
            "cy: %f, cy_error: %f, ud_out: %f, "
            "area: %f, area_error: %f, fb_out: %f",
            dt,
            cx, cx_error, pub_tra.yawspeed,
            cy, cy_error, pub_tra.velocity[2],
            area, area_error, fb_output
        );
    }else {
        pub_tra.position = m_pid.last_postition;
    }
}
