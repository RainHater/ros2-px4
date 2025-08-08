#include "control_interface/movement.h"
#include <yaml-cpp/yaml.h>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;

Movement::Movement()
{   
    m_log_name = "控制无人机(movement.cpp)";
    m_land.state = 0;
}

void Movement::initialize(std::string yaml_path){
    YAML::Node config = YAML::LoadFile(yaml_path)["movement"];
    m_yaml.HORIZONTAL_DIST_THRESHOLD = config["HORIZONTAL_DIST_THRESHOLD"].as<float>();
    m_yaml.VERTICAL_DIST_THRESHOLD = config["VERTICAL_DIST_THRESHOLD"].as<float>();
    m_yaml.delta = config["delta"].as<float>();
    m_yaml.land_correction = config["land_correction"].as<float>();
    m_yaml.land_start_time = config["land_start_time"].as<int>();
}

bool Movement::move_to_gps_target(
    double lat, double lon, float alt,
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    px4_msgs::msg::VehicleGlobalPosition init_gps,
    px4_msgs::msg::VehicleOdometry sub_tra)
{     
    if (m_gps_nav.state == 0){
        float x = 0.0, y = 0.0;
        double init_lat = init_gps.lat / 1e7;
        double init_lon = init_gps.lon / 1e7;
        float init_alt = init_gps.alt / 1e3;
        
        geo_tool::gps_to_local(
        init_lat, init_lon,
            lat, lon, 
            x, y
        );

        m_gps_nav.target[0] = x;
        m_gps_nav.target[1] = y;
        m_gps_nav.target[2] = init_alt - alt;

        m_gps_nav.state = 1;
    }

    std::array<float, 3> convert_pose = m_gps_nav.target;

    float dx = convert_pose[0] - sub_tra.position[0];
    float dy = convert_pose[1] - sub_tra.position[1];
    float dz = convert_pose[2] - sub_tra.position[2];

    float horizontal_dist = std::hypot(dx, dy);
    float vertical_dist = std::abs(dz);
    bool hor_arrive = (horizontal_dist < m_yaml.HORIZONTAL_DIST_THRESHOLD);
    bool ver_arrive = (vertical_dist < m_yaml.VERTICAL_DIST_THRESHOLD);

    pub_pose.position = convert_pose;
    pub_pose.yaw = NAN;

    bool arrive = hor_arrive && ver_arrive;
    if (arrive){
        m_gps_nav.state = 0;
    }

    return arrive;
}

bool Movement::justmove(
    px4_msgs::msg::VehicleOdometry sub_pose, 
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    rclcpp::Time instant_time,
    Waypts target,
    double v = 0.5, bool auto_angle = false)
{
    if(m_justmove.state == 0) {
        m_justmove.start_pose.x = sub_pose.position[0];
        m_justmove.start_pose.y = sub_pose.position[1];
        m_justmove.start_pose.z = sub_pose.position[2];
        m_justmove.target_pose = target;

        if (sub_pose.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED){
            RCLCPP_INFO(rclcpp::get_logger(
                m_log_name),
                "当前为 NED 坐标系"
            );
        }else if (sub_pose.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD){
            RCLCPP_INFO(rclcpp::get_logger(
                m_log_name),
                "当前为 FRD 坐标系"
            );
        }
        
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
     
        m_justmove.start_time = instant_time.seconds();
        m_justmove.state = 1;
        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "控制开始"
        );
    }

    m_justmove.dt = instant_time.seconds() - m_justmove.start_time;

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
        
        pub_pose.position[0] = out_x;
        pub_pose.position[1] = out_y;
        pub_pose.position[2] = out_z;
        pub_pose.yaw = m_justmove.dw;

        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "position[0]: %f, position[1]: %f, position[2]: %f",
            pub_pose.position[0],
            pub_pose.position[1],
            pub_pose.position[2]
        );

        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "current[0]: %f, current[1]: %f, current[2]: %f",
            sub_pose.position[0],
            sub_pose.position[1],
            sub_pose.position[2]
        );
    }else {
        m_justmove.state = 0;
    }

    return arrive;
}

bool Movement::move_by_offset(
    px4_msgs::msg::VehicleOdometry sub_pose, 
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    rclcpp::Time instant_time,
    Offset target,
    double v = 0.5, double angle = 0.0)
{
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
    bool arrive = justmove(sub_pose, pub_pose, instant_time, end, v, true);

    return arrive;
}

bool Movement::change_height(
    px4_msgs::msg::VehicleOdometry sub_pose, 
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    rclcpp::Time instant_time,
    double high,
    double v = 0.5)
{
    Waypts target = {0, 0, -high};

    bool arrive = justmove(sub_pose, pub_pose, instant_time, target, v, false);

    return arrive;
}

bool Movement::land_mode(
    double v,
    ModeControl mode_control,
    rclcpp::Time instant_time,
    common_msgs::msg::ArmOffboardStatus sub_px4_mode,
    px4_msgs::msg::VehicleOdometry sub_pose,
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    common_msgs::msg::ArmOffboardStatus &pub_px4_mode,
    TopicListener<px4_msgs::msg::VehicleLocalPosition> local_position)
{   
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
                sub_px4_mode, pub_px4_mode
            );
            if (mode_control.wait_busy()){
                tf2_tool::EulerAngles angle;
                tf2_tool::get_euler_angles(sub_pose, angle);
                m_land.start_position.x = sub_pose.position[0];
                m_land.start_position.y = sub_pose.position[1];
                m_land.dw = angle.yaw;
                m_land.state = 2;
                RCLCPP_INFO(rclcpp::get_logger(
                    m_log_name), 
                    "切换模式成功, 开始降落, 起始 dist_bottom: %f",
                    local_position.get_first_msg().dist_bottom
                );
            }
            break;
        }
        case 2:{
            auto dist_bottom = local_position.get_msg().dist_bottom;
            auto dist_bottom_valid = local_position.get_msg().dist_bottom_valid;
            auto start_dist_bottom = local_position.get_first_msg().dist_bottom;
            auto baro_height = local_position.get_first_msg().dist_bottom;

            pub_pose.position[0] = m_land.start_position.x;
            pub_pose.position[1] = m_land.start_position.y;
            pub_pose.position[2] = NAN;
            pub_pose.velocity[0] = NAN;
            pub_pose.velocity[1] = NAN;
            pub_pose.velocity[2] = v;
            pub_pose.yaw = m_land.dw;
            if (dist_bottom_valid && dist_bottom < (start_dist_bottom-m_yaml.land_correction)){
                m_land.start_time = instant_time.seconds();
                m_land.state = 3;
                RCLCPP_INFO(rclcpp::get_logger(
                    m_log_name), 
                    "降落完成!"
                );
            }
            RCLCPP_INFO(rclcpp::get_logger(
                m_log_name), 
                "当前位置 dist_bottom: %f",
                dist_bottom
            );
            break;
        }
        case 3:{
            if (instant_time.seconds() - m_land.start_time >= m_yaml.land_start_time){
                pub_px4_mode.offboard = m_land.start_state.offboard;
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
