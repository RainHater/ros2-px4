#include "control_interface/track.h"

#include <cmath>
#include <array>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace track {
Track::Track() 
    : m_log(rclcpp::get_logger("跟踪(track.cpp)"))
    , m_camera({1920, 1080, 70})
{   
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["track"];
    m_yaml.yaw.kp = config["yaw_kp"].as<float>();
    m_yaml.yaw.ki = config["yaw_ki"].as<float>();
    m_yaml.yaw.kd = config["yaw_kd"].as<float>();
    m_yaml.yaw.th = config["yaw_th"].as<float>();
    m_yaml.ud.kp = config["ud_kp"].as<float>();
    m_yaml.ud.ki = config["ud_ki"].as<float>();
    m_yaml.ud.kd = config["ud_kd"].as<float>();
    m_yaml.ud.th = config["ud_th"].as<float>();
    m_yaml.fb.kp = config["fb_kp"].as<float>();
    m_yaml.fb.ki = config["fb_ki"].as<float>();
    m_yaml.fb.kd = config["fb_kd"].as<float>();
    m_yaml.fb.th = config["fb_th"].as<float>();
    m_yaml.area_th = config["area_th"].as<float>();
    m_yaml.is_filter = config["is_filter"].as<bool>();

    RCLCPP_INFO(m_log, "-----------跟踪配置文件----------");
    RCLCPP_INFO(m_log, "左右pid, kp: %f, ki: %f, kd: %f, th: %f", m_yaml.yaw.kp, m_yaml.yaw.ki, m_yaml.yaw.kd, m_yaml.yaw.th);
    RCLCPP_INFO(m_log, "上下pid, kp: %f, ki: %f, kd: %f, th: %f", m_yaml.ud.kp, m_yaml.ud.ki, m_yaml.ud.kd, m_yaml.ud.th);
    RCLCPP_INFO(m_log, "前后pid, kp: %f, ki: %f, kd: %f, th: %f", m_yaml.fb.kp, m_yaml.fb.ki, m_yaml.fb.kd, m_yaml.fb.th);
    RCLCPP_INFO(m_log, "跟踪面积阈值: %f", m_yaml.area_th);
    RCLCPP_INFO(m_log, "是否过滤: %d", m_yaml.is_filter);
    RCLCPP_INFO(m_log, "-----------配置文件结束----------");

    m_normal_track.thre_area = m_camera.width*m_camera.height*m_yaml.area_th;

    FilterType is_filter = FilterType::None;
    if (m_yaml.is_filter){
        is_filter = FilterType::LowPass2;
    }

    m_normal_track.yaw.initialize(
        m_yaml.yaw.kp, 
        m_yaml.yaw.ki, 
        m_yaml.yaw.kd, 
        false, 
        m_yaml.yaw.th, 
        m_yaml.yaw.th, 
        0.05f, 
        0.02f, 
        is_filter, 
        is_filter
    ); 
    m_normal_track.ud.initialize(
        m_yaml.ud.kp, 
        m_yaml.ud.ki, 
        m_yaml.ud.kd, 
        false, 
        m_yaml.ud.th, 
        m_yaml.ud.th, 
        0.05f, 
        0.02f, 
        is_filter, 
        is_filter
    ); 
    m_normal_track.fb.initialize(
        m_yaml.fb.kp, 
        m_yaml.fb.ki, 
        m_yaml.fb.kd, 
        false, 
        m_yaml.fb.th, 
        m_yaml.fb.th, 
        0.05f, 
        0.02f, 
        is_filter, 
        is_filter
    );
}

void Track::normalTrack(
    bool is_target_valid,
    int64_t dt,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    std::vector<identify::msg::YoloDetection> det_targets,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    auto cur_yaw = utilities::convert::flo_to_yaw(flo_q);

    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = cur_yaw;

    if (is_target_valid){
        auto detection = det_targets[0];
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

    float yaw_output = m_normal_track.yaw.compute(cx_error, dt);
    float ud_output = m_normal_track.ud.compute(cy_error, dt);
    float fb_output = m_normal_track.fb.compute(area_error, dt);

    pub_pos_msgs.yaw = NAN;
    pub_pos_msgs.yawspeed = yaw_output;
    pub_pos_msgs.position[0] = NAN;
    pub_pos_msgs.position[1] = NAN;
    pub_pos_msgs.position[2] = NAN;
    pub_pos_msgs.velocity[0] = cosf(yaw) * fb_output;
    pub_pos_msgs.velocity[1] = sinf(yaw) * fb_output;
    pub_pos_msgs.velocity[2] = ud_output;

    if (is_target_valid){
        RCLCPP_INFO(m_log, "-----------跟踪----------");
        RCLCPP_INFO(m_log, "dt: %ld", dt);
        RCLCPP_INFO(m_log, "cx: %f, cx_error: %f, yaw_out: %f", cx, cx_error, pub_pos_msgs.yawspeed);
        RCLCPP_INFO(m_log, "cy: %f, cy_error: %f, ud_out: %f", cy, cy_error, ud_output);
        RCLCPP_INFO(m_log, "area: %f, area_error: %f, fb_out: %f", area, area_error, fb_output);
        RCLCPP_INFO(m_log, "cur_yaw: %f", yaw);
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }else {
        RCLCPP_INFO(m_log, "-----------未跟踪----------");
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos);
        log_printf_tool::printf_log_cur_vec(m_log, pub_pos_msgs.velocity);
        RCLCPP_INFO(m_log, "-------------------------");
    }
}

void Track::normalTrack_v1(
    bool is_target_valid,
    int64_t dt,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    std::vector<identify::msg::YoloDetection> det_targets,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    auto detect_flag = is_target_valid;
    auto cur_yaw = utilities::convert::flo_to_yaw(flo_q);

    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = cur_yaw;

    if (detect_flag){
        auto detection = det_targets[0];
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

    float yaw_output = m_normal_track.yaw.compute(cx_error, dt);
    float ud_output = m_normal_track.ud.compute(cy_error, dt);
    float fb_output = m_normal_track.fb.compute(area_error, dt);

    pub_pos_msgs.yaw = NAN;
    pub_pos_msgs.yawspeed = yaw_output;

    if (!m_normal_track.last_pos_init){
        m_normal_track.last_pos = cur_pos;
        m_normal_track.last_pos_init = true;
        log_printf_tool::printf_log_title_pos(m_log, "上一次位置", m_normal_track.last_pos);
    }

    if (detect_flag){
        m_normal_track.last_pos = cur_pos;
        m_normal_track.last_pos_update = true;
        pub_pos_msgs.position[0] = NAN;
        pub_pos_msgs.position[1] = NAN;
        pub_pos_msgs.position[2] = NAN;
        pub_pos_msgs.velocity[0] = cosf(yaw) * fb_output;
        pub_pos_msgs.velocity[1] = sinf(yaw) * fb_output;
        pub_pos_msgs.velocity[2] = ud_output;

        RCLCPP_INFO(m_log, "--跟踪-------------------");
        RCLCPP_INFO(m_log, "dt: %ld", dt);
        RCLCPP_INFO(m_log, "cx: %f, cx_error: %f, yaw_out: %f", cx, cx_error, yaw_output);
        RCLCPP_INFO(m_log, "cy: %f, cy_error: %f, ud_out: %f", cy, cy_error, ud_output);
        RCLCPP_INFO(m_log, "area: %f, area_error: %f, fb_out: %f", area, area_error, fb_output);
        RCLCPP_INFO(m_log, "cur_yaw: %f", yaw);
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }else {
        m_normal_track.last_pos_update = false;
        pub_pos_msgs.position = m_normal_track.last_pos;
        RCLCPP_INFO(m_log, "--未跟踪-------------------");
        log_printf_tool::printf_log_pos(m_log, pub_pos_msgs.position, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }
}

void Track::normalTrack_v2(
    bool is_target_valid,
    int64_t dt,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    std::vector<identify::msg::YoloDetection> det_targets,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    auto detect_flag = is_target_valid;
    auto cur_yaw = utilities::convert::flo_to_yaw(flo_q);


    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = cur_yaw;

    auto detection = det_targets[0];
    if (detect_flag){
        m_normal_track.last_detection = detection;
    }else {
        detection = m_normal_track.last_detection;
    }
    cx = detection.cx;
    cy = detection.cy;
    area = abs((detection.x_max - detection.x_min) * (detection.y_max - detection.y_min));  

    float cx_error = cx - m_camera.width / 2.0;
    float cy_error = cy - m_camera.height / 2.0;
    float area_error = m_normal_track.thre_area - area;

    float yaw_output = m_normal_track.yaw.compute(cx_error, dt);
    float ud_output = m_normal_track.ud.compute(cy_error, dt);
    float fb_output = m_normal_track.fb.compute(area_error, dt);

    pub_pos_msgs.yaw = NAN;
    pub_pos_msgs.yawspeed = yaw_output;

    if (!m_normal_track.last_pos_init){
        m_normal_track.last_pos = cur_pos;
        m_normal_track.last_pos_init = true;
        log_printf_tool::printf_log_title_pos(m_log, "上一次位置", m_normal_track.last_pos);
    }

    if (detect_flag){
        m_normal_track.last_pos = cur_pos;
        m_normal_track.last_pos_update = true;
        pub_pos_msgs.position[0] = NAN;
        pub_pos_msgs.position[1] = NAN;
        pub_pos_msgs.position[2] = NAN;
        pub_pos_msgs.velocity[0] = cosf(yaw) * fb_output;
        pub_pos_msgs.velocity[1] = sinf(yaw) * fb_output;
        pub_pos_msgs.velocity[2] = ud_output;

        RCLCPP_INFO(m_log, "--跟踪-------------------");
        RCLCPP_INFO(m_log, "dt: %ld", dt);
        RCLCPP_INFO(m_log, "cur_yaw: %f", yaw);
        RCLCPP_INFO(m_log, "cx: %f, cx_error: %f, yaw_out: %f", cx, cx_error, yaw_output);
        RCLCPP_INFO(m_log, "cy: %f, cy_error: %f, ud_out: %f", cy, cy_error, ud_output);
        RCLCPP_INFO(m_log, "area: %f, area_error: %f, fb_out: %f", area, area_error, fb_output);
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }else {
        m_normal_track.last_pos_update = false;
        pub_pos_msgs.position = m_normal_track.last_pos;
        RCLCPP_INFO(m_log, "--未跟踪-------------------");
        log_printf_tool::printf_log_pos(m_log, pub_pos_msgs.position, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }
}

void Track::normalTrack_v3(
    bool is_target_valid,
    int64_t dt,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    std::vector<identify::msg::YoloDetection> det_targets,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    auto detect_flag = is_target_valid;
    auto cur_yaw = utilities::convert::flo_to_yaw(flo_q);

    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = cur_yaw;

    auto detection = det_targets[0];
    if (detect_flag){
        m_normal_track.last_detection = detection;
    }else {
        detection = m_normal_track.last_detection;
    }
    cx = detection.cx;
    cy = detection.cy;
    area = abs((detection.x_max - detection.x_min) * (detection.y_max - detection.y_min));  

    float cx_error = cx - m_camera.width / 2.0;
    float cy_error = cy - m_camera.height / 2.0;
    float area_error = m_normal_track.thre_area - area;

    float yaw_output = m_normal_track.yaw.compute(cx_error, dt);
    float ud_output = m_normal_track.ud.compute(cy_error, dt);
    float fb_output = m_normal_track.fb.compute(area_error, dt);

    pub_pos_msgs.yaw = NAN;
    pub_pos_msgs.yawspeed = yaw_output;
    pub_pos_msgs.position[0] = NAN;
    pub_pos_msgs.position[1] = NAN;
    pub_pos_msgs.position[2] = NAN;
    pub_pos_msgs.velocity[0] = cosf(yaw) * fb_output;
    pub_pos_msgs.velocity[1] = sinf(yaw) * fb_output;
    pub_pos_msgs.velocity[2] = ud_output;

    if (detect_flag){
        RCLCPP_INFO(m_log, "-----------跟踪----------");
        RCLCPP_INFO(m_log, "dt: %ld", dt);
        RCLCPP_INFO(m_log, "cx: %f, cx_error: %f, yaw_out: %f", cx, cx_error, yaw_output);
        RCLCPP_INFO(m_log, "cy: %f, cy_error: %f, ud_out: %f", cy, cy_error, ud_output);
        RCLCPP_INFO(m_log, "area: %f, area_error: %f, fb_out: %f", area, area_error, fb_output);
        RCLCPP_INFO(m_log, "cur_yaw: %f", yaw);
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos); 
        RCLCPP_INFO(m_log, "-------------------------");
    }else {
        RCLCPP_INFO(m_log, "-----------未跟踪----------");
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos); 
        log_printf_tool::printf_log_cur_vec(m_log, pub_pos_msgs.velocity);
        RCLCPP_INFO(m_log, "-------------------------");
    }
}

void Track::normalTrack_v4(
    bool is_target_valid,
    int64_t dt,
    std::array<float, 3> cur_pos,
    std::array<float, 3> g_pos,
    std::array<float, 4> flo_q,
    std::vector<identify::msg::YoloDetection> det_targets,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    auto detect_flag = is_target_valid;
    auto cur_yaw = utilities::convert::flo_to_yaw(flo_q);

    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = cur_yaw;

    if (detect_flag){
        auto detection = det_targets[0];
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

    float yaw_output = m_normal_track.yaw.compute(cx_error, dt);
    float ud_output = m_normal_track.ud.compute(cy_error, dt);
    float fb_output = m_normal_track.fb.compute(area_error, dt);

    pub_pos_msgs.yaw = NAN;
    pub_pos_msgs.yawspeed = yaw_output;
    pub_pos_msgs.position[0] = NAN;
    pub_pos_msgs.position[1] = NAN;
    pub_pos_msgs.position[2] = NAN;
    pub_pos_msgs.velocity[0] = cosf(yaw) * fb_output;
    pub_pos_msgs.velocity[1] = sinf(yaw) * fb_output;
    pub_pos_msgs.velocity[2] = ud_output;

    if (detect_flag){
        RCLCPP_INFO(m_log, "--跟踪-------------------");
        RCLCPP_INFO(m_log, "dt: %ld", dt);
        RCLCPP_INFO(m_log, "cx: %f, cx_error: %f, yaw_out: %f", cx, cx_error, yaw_output);
        RCLCPP_INFO(m_log, "cy: %f, cy_error: %f, ud_out: %f", cy, cy_error, ud_output);
        RCLCPP_INFO(m_log, "area: %f, area_error: %f, fb_out: %f", area, area_error, fb_output);
        RCLCPP_INFO(m_log, "cur_yaw: %f", yaw);
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }else {
        pub_pos_msgs.position = g_pos;
        RCLCPP_INFO(m_log, "--未跟踪-------------------");
        log_printf_tool::printf_log_pos(m_log, pub_pos_msgs.position, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }
}

void Track::normalTrack_v5(
    bool is_target_valid,
    int64_t dt,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    std::vector<identify::msg::YoloDetection> det_targets,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    auto detect_flag = is_target_valid;
    auto cur_yaw = utilities::convert::flo_to_yaw(flo_q);

    float cx = 0.0f;
    float cy = 0.0f;
    float area = 0.0f;
    float yaw = cur_yaw;

    if (detect_flag){
        auto detection = det_targets[0];
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

    float yaw_output = m_normal_track.yaw.compute(cx_error, dt);
    float ud_output = m_normal_track.ud.compute(cy_error, dt);
    float fb_output = m_normal_track.fb.compute(area_error, dt);

    float vec_0 = cosf(yaw) * fb_output;
    float vec_1 = sinf(yaw) * fb_output;
    float vec_2 = ud_output;

    float pos_0 = pub_pos_msgs.position[0] + vec_0 * (dt/1000.0f);
    float pos_1 = pub_pos_msgs.position[1] + vec_1 * (dt/1000.0f);
    float pos_2 = pub_pos_msgs.position[2] + vec_2 * (dt/1000.0f);

    pub_pos_msgs.yaw = NAN;
    pub_pos_msgs.yawspeed = yaw_output;
    pub_pos_msgs.position[0] = pos_0;
    pub_pos_msgs.position[1] = pos_1;
    pub_pos_msgs.position[2] = pos_2;
    pub_pos_msgs.velocity[0] = vec_0;
    pub_pos_msgs.velocity[1] = vec_1;
    pub_pos_msgs.velocity[2] = vec_2;

    if (detect_flag){
        RCLCPP_INFO(m_log, "--跟踪-------------------");
        RCLCPP_INFO(m_log, "dt: %ld", dt);
        RCLCPP_INFO(m_log, "cx: %f, cx_error: %f, yaw_out: %f", cx, cx_error, yaw_output);
        RCLCPP_INFO(m_log, "cy: %f, cy_error: %f, ud_out: %f", cy, cy_error, ud_output);
        RCLCPP_INFO(m_log, "area: %f, area_error: %f, fb_out: %f", area, area_error, fb_output);
        RCLCPP_INFO(m_log, "cur_yaw: %f", yaw);
        log_printf_tool::printf_log_cur_pos(m_log, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }else {
        RCLCPP_INFO(m_log, "--未跟踪-------------------");
        log_printf_tool::printf_log_pos(m_log, pub_pos_msgs.position, cur_pos);
        RCLCPP_INFO(m_log, "-------------------------");
    }
}

void Track::updateLastPostition(std::array<float, 3> pos){
    if (m_normal_track.last_pos_update){
        m_normal_track.last_pos = pos;
    }
}
}
