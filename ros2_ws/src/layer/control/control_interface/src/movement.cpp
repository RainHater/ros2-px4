#include "control_interface/movement.h"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;

Movement::Movement()
 : m_log(rclcpp::get_logger("控制无人机(movement.cpp)"))
{    
    m_land.state = 0;

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

bool Movement::move_to_gps_target(movement::MoveToGPSTarget msgs_info) {
    auto& init = msgs_info.init;
    // auto& origin = msgs_info.origin;
    auto& target = msgs_info.target;
    auto& sub_tra = msgs_info.sub_tra;
    auto& pub_pose = msgs_info.pub_pose;

    if (m_gps_nav.state == 0){
        float x = 0.0, y = 0.0;
        double init_lat = init.lat;
        double init_lon = init.lon;
        double init_alt = init.alt;
        double target_lat = target.lat;
        double target_lon = target.lon;
        double target_alt = target.alt;
        
        geo_tool::gps_to_local(
        init_lat, init_lon,
            target_lat, target_lon, 
            x, y
        );

        m_gps_nav.target[0] = x;
        m_gps_nav.target[1] = y;
        m_gps_nav.target[2] = init_alt - target_alt;

        m_gps_nav.state = 1;
    }

    std::array<float, 3> convert_pose = m_gps_nav.target;

    float dx = convert_pose[0] - sub_tra.position[0];
    float dy = convert_pose[1] - sub_tra.position[1];
    float dz = convert_pose[2] - sub_tra.position[2];

    float horizontal_dist = std::hypot(dx, dy);
    float vertical_dist = std::abs(dz);
    bool hor_arrive = (horizontal_dist < m_yaml.hor_th);
    bool ver_arrive = (vertical_dist < m_yaml.ver_th);

    pub_pose->position = convert_pose;
    pub_pose->yaw = NAN;

    bool arrive = hor_arrive && ver_arrive;
    if (arrive){
        m_gps_nav.state = 0;
    }

    return arrive;
}

bool Movement::justmove(movement::JustmoveInfo justmove_info, Waypts target) {   
    auto& sub_pose = justmove_info.sub_pose;
    auto& pub_pose = justmove_info.pub_pose;
    auto& instant_time = justmove_info.instant_time;
    auto& v = justmove_info.v;
    auto& auto_angle = justmove_info.auto_angle;
    auto now_s = instant_time.seconds();

    if(m_justmove.state == 0) {
        m_justmove.start_pose.x = sub_pose.position[0];
        m_justmove.start_pose.y = sub_pose.position[1];
        m_justmove.start_pose.z = sub_pose.position[2];
        m_justmove.target_pose = target;

        double distancex = m_justmove.target_pose.x - m_justmove.start_pose.x;
        double distancey = m_justmove.target_pose.y - m_justmove.start_pose.y;
        double distancez = m_justmove.target_pose.z - m_justmove.start_pose.z;
        double distance  = sqrt(pow(distancex,2) + pow(distancey,2) + pow(distancez,2));

        m_justmove.total_time = distance / v;
        m_justmove.vx = distancex / m_justmove.total_time;
        m_justmove.vy = distancey / m_justmove.total_time;
        m_justmove.vz = distancez / m_justmove.total_time;
        if (auto_angle){
            m_justmove.dw = atan2(m_justmove.vy, m_justmove.vx);
        }else {
            tf2_tool::EulerAngles angle;
            tf2_tool::get_euler_angles(sub_pose, angle);
            m_justmove.dw = angle.yaw;
        }
     
        m_justmove.start_time = now_s;
        m_justmove.state = 1;
        RCLCPP_INFO(m_log, 
            "控制开始"
        );
    }

    m_justmove.dt = now_s - m_justmove.start_time;

    double deltax = sub_pose.position[0]-m_justmove.target_pose.x;
    double deltay = sub_pose.position[1]-m_justmove.target_pose.y;
    double deltaz = sub_pose.position[2]-m_justmove.target_pose.z;

    double delta = sqrt(pow(deltax,2)+pow(deltay,2)+pow(deltaz,2));
    bool arrive = (instant_time.seconds() - m_justmove.start_time > m_justmove.total_time) && delta < m_yaml.delta;

    if(!arrive){
        double out_x = m_justmove.start_pose.x + m_justmove.dt * m_justmove.vx;
        double out_y = m_justmove.start_pose.y + m_justmove.dt * m_justmove.vy;
        double out_z = m_justmove.start_pose.z + m_justmove.dt * m_justmove.vz;

        auto limit_to_destination = [](double out, double v, double dest) -> double {
            if ((v > 0 && out > dest) || (v < 0 && out < dest)) {
                return dest;
            }
            return out;
        };

        out_x = limit_to_destination(out_x, m_justmove.vx, m_justmove.target_pose.x);
        out_y = limit_to_destination(out_y, m_justmove.vy, m_justmove.target_pose.y);
        out_z = limit_to_destination(out_z, m_justmove.vz, m_justmove.target_pose.z);
        
        pub_pose->position[0] = out_x;
        pub_pose->position[1] = out_y;
        pub_pose->position[2] = out_z;
        pub_pose->yaw = m_justmove.dw;

        RCLCPP_INFO(m_log, 
            "position[0]: %f, position[1]: %f, position[2]: %f, "
            "current[0]: %f, current[1]: %f, current[2]: %f",
            pub_pose->position[0],
            pub_pose->position[1],
            pub_pose->position[2],
            sub_pose.position[0],
            sub_pose.position[1],
            sub_pose.position[2]
        );
    }else {
        m_justmove.state = 0;
    }

    return arrive;
}

bool Movement::justmove_outdoor(movement::JustmoveInfo justmove_info, Waypts target){
    auto& sub_pose = justmove_info.sub_pose;
    auto& pub_pose = justmove_info.pub_pose;
    auto& auto_angle = justmove_info.auto_angle;

    if (auto_angle){
        m_justmove.dw = atan2(m_justmove.vy, m_justmove.vx);
    }else {
        tf2_tool::EulerAngles angle;
        tf2_tool::get_euler_angles(sub_pose, angle);
        m_justmove.dw = angle.yaw;
    }
    
    pub_pose->position[0] = target.x;
    pub_pose->position[1] = target.y;
    pub_pose->position[2] = target.z;
    pub_pose->yaw = m_justmove.dw;

    double deltax = sub_pose.position[0]-target.x;
    double deltay = sub_pose.position[1]-target.y;
    double deltaz = sub_pose.position[2]-target.z;

    float horizontal_dist = std::hypot(deltax, deltay);
    float vertical_dist = std::abs(deltaz);
    bool hor_arrive = (horizontal_dist < m_yaml.hor_th);
    bool ver_arrive = (vertical_dist < m_yaml.ver_th);

    bool arrive = hor_arrive && ver_arrive;

    RCLCPP_INFO(m_log, 
        "position[0]: %f, position[1]: %f, position[2]: %f, "
        "current[0]: %f, current[1]: %f, current[2]: %f",
        pub_pose->position[0],
        pub_pose->position[1],
        pub_pose->position[2],
        sub_pose.position[0],
        sub_pose.position[1],
        sub_pose.position[2]
    );

    return arrive;
}

bool Movement::move_by_offset(
    movement::JustmoveInfo justmove_info, 
    Offset target,
    double angle)
{
    auto& sub_pose = justmove_info.sub_pose;

    tf2_tool::EulerAngles angles;
    tf2_tool::get_euler_angles(sub_pose, angles);
    float yaw = geo_tool::normalize_angle(angles.yaw + geo_tool::deg2rad(angle));
    float dx = target.forward * std::cos(yaw) - target.right * std::sin(yaw);
    float dy = target.forward * std::sin(yaw) + target.right * std::cos(yaw);
    float dz = -target.up;

    Waypts end;
    end.x = (sub_pose.position[0] + dx);
    end.y = (sub_pose.position[1] + dy);
    end.z = (sub_pose.position[2] + dz);

    justmove_info.auto_angle = true;

    bool arrive = justmove(justmove_info, end);

    return arrive;
}

bool Movement::change_height(movement::JustmoveInfo justmove_info, double high, bool outdoor){
    auto& sub_pose = justmove_info.sub_pose;
    auto c_x = sub_pose.position[0];
    auto c_y = sub_pose.position[1];
    auto c_z = sub_pose.position[2];
    justmove_info.auto_angle = false;

    bool arrive = false;
    if (m_change_height.state==0){
        m_change_height.start_pos = {c_x, c_y, c_z + (-high)};
        m_change_height.state = 1;
    }  

    if (outdoor){
        arrive = justmove_outdoor(justmove_info, m_change_height.start_pos);
    }else {
        arrive = justmove(justmove_info, m_change_height.start_pos);
    }

    if (arrive){
       m_change_height.state = 0;
    } 

    return arrive;
}

bool Movement::land_mode(movement::LandModeInfo land_mode_info, float v) {   
    auto& pub_pose = land_mode_info.pub_pose;
    auto& pub_px4_mode = land_mode_info.pub_px4_mode;
    auto& sub_pose = land_mode_info.sub_pose;
    auto& sub_px4_mode = land_mode_info.sub_px4_mode;
    auto& mode_control = land_mode_info.mode_control;
    auto& local_position = land_mode_info.local_position;
    auto& instant_time = land_mode_info.instant_time;
    
    bool finish = false;

    switch(m_land.state){
        case 0:{
            m_land.start_state = sub_px4_mode;
            m_land.state = 1;
            break;
        }
        case 1:{
            mode_control.unlock(
                ARM_ENABLE,
                (POSITION | VELOCITY),
                sub_px4_mode, *pub_px4_mode
            );
            if (mode_control.wait_busy()){
                tf2_tool::EulerAngles angle;
                tf2_tool::get_euler_angles(sub_pose, angle);
                m_land.start_position.x = sub_pose.position[0];
                m_land.start_position.y = sub_pose.position[1];
                m_land.dw = angle.yaw;
                m_land.state = 2;
                RCLCPP_INFO(m_log, 
                    "切换模式成功, 开始降落, 起始 dist_bottom: %f",
                    local_position.get_first_msg().dist_bottom
                );
            }
            break;
        }
        case 2:{
            // auto dist_bottom = local_position.get_msg().dist_bottom;
            auto dist_bittom = -sub_pose.position[2];
            // auto dist_bottom_valid = local_position.get_msg().dist_bottom_valid;
            // auto start_dist_bottom = local_position.get_first_msg().dist_bottom;

            pub_pose->position[0] = m_land.start_position.x;
            pub_pose->position[1] = m_land.start_position.y;
            pub_pose->position[2] = 0.0;
            pub_pose->velocity[0] = NAN;
            pub_pose->velocity[1] = NAN;
            pub_pose->velocity[2] = v;
            pub_pose->yaw = m_land.dw;
            // if (dist_bittom < m_yaml.land_th){
            //     m_land.start_time = instant_time.seconds();
            //     m_land.state = 3;
            //     RCLCPP_INFO(m_log, 
            //         "降落完成!"
            //     );
            // }
            RCLCPP_INFO(m_log, 
                "当前位置 dist_bottom: %f",
                dist_bittom
            );
            break;
        }
        case 3:{
            if (instant_time.seconds() - m_land.start_time >= m_yaml.land_start_time){
                pub_px4_mode->offboard = m_land.start_state.offboard;
                m_land.state = 4;
            }
            break;
        }
        case 4:{
            finish = true;
            m_land.state = 0;
            break;
        }
    }
    return finish;
}
