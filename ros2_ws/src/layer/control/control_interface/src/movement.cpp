#include "control_interface/movement.h"

#include <array>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace movement{
Movement::Movement()
 : m_log(rclcpp::get_logger("控制无人机(movement.cpp)"))
{    
    std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/app.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path)["movement"];
    m_yaml.hor_th = config["hor_th"].as<float>();
    m_yaml.ver_th = config["ver_th"].as<float>();
    m_yaml.delta = config["delta"].as<float>();
    m_yaml.land_correction = config["land_correction"].as<float>();
    m_yaml.land_start_time = config["land_start_time"].as<float>();
    m_yaml.move_time_out = config["move_time_out"].as<float>();
    m_yaml.land_th = config["land_th"].as<float>();
}

bool Movement::nav_move_to_target(
    geo_tool::GeoCoordinate target_nav,
    geo_tool::GeoCoordinate start_nav,
    std::array<float, 3> cur_pos,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {
    bool arrive = false;

    switch(m_move_nav.state){
        case nav_move_to_target::IDLE: {
            float x = 0.0, y = 0.0;
            double init_lat = start_nav.lat;
            double init_lon = start_nav.lon;
            double init_alt = start_nav.alt;
            double target_lat = target_nav.lat;
            double target_lon = target_nav.lon;
            double target_alt = target_nav.alt;

            geo_tool::gps_to_local(
            init_lat, init_lon,
                target_lat, target_lon, 
                x, y
            );

            m_move_nav.target_nav[0] = x;
            m_move_nav.target_nav[1] = y;
            m_move_nav.target_nav[2] = init_alt - target_alt;

            m_move_nav.state = nav_move_to_target::FLY;
            break;
        }
        case nav_move_to_target::FLY: {
            std::array<float, 3> convert_pose = m_move_nav.target_nav;
            float dx = convert_pose[0] - cur_pos[0];
            float dy = convert_pose[1] - cur_pos[1];
            float dz = convert_pose[2] - cur_pos[2];

            float horizontal_dist = std::hypot(dx, dy);
            float vertical_dist = std::abs(dz);
            bool hor_arrive = (horizontal_dist < m_yaml.hor_th);
            bool ver_arrive = (vertical_dist < m_yaml.ver_th);

            pub_pos_msgs.position = convert_pose;
            pub_pos_msgs.yaw = NAN;

            arrive = hor_arrive && ver_arrive;
            if (arrive){
                m_move_nav.state = nav_move_to_target::IDLE;
            }
            break;
        }
    }
    return arrive;
}

bool Movement::justmove(
    std::array<float, 3> target_pos,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    rclcpp::Time instant_time,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
    bool auto_angle,
    float v
){
    auto now_s = instant_time.seconds();
    auto arrive = false;

    switch(m_justmove.state){
        case justmove::IDLE: {
            m_justmove.start_pos = cur_pos;
            m_justmove.target_pos = target_pos;

            double distancex = m_justmove.target_pos[0] - m_justmove.start_pos[0];
            double distancey = m_justmove.target_pos[1] - m_justmove.start_pos[1];
            double distancez = m_justmove.target_pos[2] - m_justmove.start_pos[2];
            double distance  = sqrt(pow(distancex,2) + pow(distancey,2) + pow(distancez,2));

            m_justmove.total_time = distance / v;
            m_justmove.vx = distancex / m_justmove.total_time;
            m_justmove.vy = distancey / m_justmove.total_time;
            m_justmove.vz = distancez / m_justmove.total_time;

            if (auto_angle){
                m_justmove.dw = atan2(m_justmove.vy, m_justmove.vx);
            }else {
                m_justmove.dw = tf2_tool::flo_to_yaw(flo_q);
            }

            m_justmove.start_time = now_s;
            m_justmove.state = justmove::FLY;

            RCLCPP_INFO(m_log, 
                "justmove IDLE 计算完成"
            );

            break;
        }
        case justmove::FLY: {
            m_justmove.dt = now_s - m_justmove.start_time;

            double deltax = cur_pos[0]-m_justmove.target_pos[0];
            double deltay = cur_pos[1]-m_justmove.target_pos[1];
            double deltaz = cur_pos[2]-m_justmove.target_pos[2];

            double delta = sqrt(pow(deltax,2)+pow(deltay,2)+pow(deltaz,2));
            arrive = (instant_time.seconds() - m_justmove.start_time > m_justmove.total_time) && delta < m_yaml.delta;

            if(!arrive){
                double out_x = m_justmove.start_pos[0] + m_justmove.dt * m_justmove.vx;
                double out_y = m_justmove.start_pos[1] + m_justmove.dt * m_justmove.vy;
                double out_z = m_justmove.start_pos[2] + m_justmove.dt * m_justmove.vz;

                auto limit_to_destination = [](double out, double v, double dest) -> double {
                    if ((v > 0 && out > dest) || (v < 0 && out < dest)) {
                        return dest;
                    }
                    return out;
                };

                out_x = limit_to_destination(out_x, m_justmove.vx, m_justmove.target_pos[0]);
                out_y = limit_to_destination(out_y, m_justmove.vy, m_justmove.target_pos[1]);
                out_z = limit_to_destination(out_z, m_justmove.vz, m_justmove.target_pos[2]);
                
                pub_pos_msgs.position[0] = out_x;
                pub_pos_msgs.position[1] = out_y;
                pub_pos_msgs.position[2] = out_z;
                pub_pos_msgs.yaw = m_justmove.dw;

                log_printf_tool::printf_log_pos(m_log, pub_pos_msgs.position, cur_pos);
            }else {
                m_justmove.state = justmove::IDLE;
            }
            break;
        }
    }

    return arrive;
}

bool Movement::justmove_outdoor(
    std::array<float, 3> target_pos,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
    bool auto_angle
){  
    if (auto_angle){
        m_justmove.dw = atan2(m_justmove.vy, m_justmove.vx);
    }else {
        m_justmove.dw = tf2_tool::flo_to_yaw(flo_q);
    }

    pub_pos_msgs.position = target_pos;
    pub_pos_msgs.yaw = m_justmove.dw;

    double deltax = cur_pos[0]-target_pos[0];
    double deltay = cur_pos[1]-target_pos[1];
    double deltaz = cur_pos[2]-target_pos[2];

    float horizontal_dist = std::hypot(deltax, deltay);
    float vertical_dist = std::abs(deltaz);
    bool hor_arrive = (horizontal_dist < m_yaml.hor_th);
    bool ver_arrive = (vertical_dist < m_yaml.ver_th);

    bool arrive = hor_arrive && ver_arrive;

    log_printf_tool::printf_log_pos(m_log, pub_pos_msgs.position, cur_pos);

    return arrive;
}

bool Movement::move_by_offset(
    std::array<float, 3> target_pos,
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    rclcpp::Time instant_time,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
    float angle,
    float v,
    bool outdoor
){
    bool arrive = false;

    switch(m_move_by_offset_info.cur_step){
        case move_by_offset::IDLE: {
            auto cur_yaw = tf2_tool::flo_to_yaw(flo_q);
            float yaw = geo_tool::normalize_angle(cur_yaw + geo_tool::deg2rad(angle));
            float dx = target_pos[0] * std::cos(yaw) - target_pos[1] * std::sin(yaw);
            float dy = target_pos[0] * std::sin(yaw) + target_pos[1] * std::cos(yaw);
            float dz = -target_pos[2];

            std::array<float, 3> cal_pos;
            cal_pos[0] = cur_pos[0] + dx;
            cal_pos[1] = cur_pos[1] + dy;
            cal_pos[2] = cur_pos[2] + dz;
            m_move_by_offset_info.cal_pos = cal_pos;
            m_move_by_offset_info.cur_step = move_by_offset::FLY;
            break;
        }
        case move_by_offset::FLY: {
            if (outdoor){
                justmove_outdoor(
                    m_move_by_offset_info.cal_pos,
                    cur_pos,
                    flo_q,
                    pub_pos_msgs,
                    true
                );
            }else {
                arrive = justmove(
                    m_move_by_offset_info.cal_pos,
                    cur_pos,
                    flo_q,
                    instant_time,
                    pub_pos_msgs,
                    true,
                    v
                );
            }

            if (arrive){
                m_move_by_offset_info.cur_step = move_by_offset::IDLE;
            }
            break;
        }
    }

    return arrive;
}


bool Movement::change_height(
    std::array<float, 3> cur_pos,
    std::array<float, 4> flo_q,
    rclcpp::Time instant_time,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
    float high, 
    float v,
    bool outdoor
){  
    bool arrive = false;

    switch(m_change_height.state){
        case change_height::IDLE:{
            m_change_height.start_pos = {cur_pos[0], cur_pos[1], cur_pos[2] + (-high)};
            m_change_height.state = change_height::FLY;
            break;
        }
        case change_height::FLY:{
            if (outdoor){
                arrive = justmove_outdoor(
                    m_change_height.start_pos,
                    cur_pos,
                    flo_q,
                    pub_pos_msgs,
                    true
                );
            }else {
                arrive = justmove(
                    m_change_height.start_pos,
                    cur_pos,
                    flo_q,
                    instant_time,
                    pub_pos_msgs,
                    true,
                    v
                );
            }
            if (arrive){
                m_change_height.state = change_height::IDLE;
            }
            break;
        }
    }

    return arrive;
}

void Movement::land_mode(
    std::array<float, 4> flo_q,
    px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
) {     
    auto cur_yaw = tf2_tool::flo_to_yaw(flo_q);

    pub_pos_msgs.position[0] = NAN;
    pub_pos_msgs.position[1] = NAN;
    pub_pos_msgs.position[2] = NAN;
    pub_pos_msgs.velocity[0] = 0.0f;
    pub_pos_msgs.velocity[1] = 0.0f;
    pub_pos_msgs.velocity[2] = 0.5f;
    pub_pos_msgs.yaw = cur_yaw;
}
};
