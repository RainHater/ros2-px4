#include "control_interface/movement.h"
#include <rclcpp/logging.hpp>
#include <utilities/geo_tool.hpp>
#include <utilities/tf2_tool.hpp>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;

Movement::Movement()
{
    m_log_name = "控制无人机(movement.cpp)";
    m_land.state = 0;
}

bool Movement::wait_busy() const{
    if (!m_states.indicator)
        return true;
    return false;
}

void Movement::switchflymode(
    rclcpp::Time now, 
    px4_msgs::msg::VehicleOdometry current)
{   
    double deltax = current.position[0]-m_local.destination.x;
    double deltay = current.position[1]-m_local.destination.y;
    double deltaz = current.position[2]-m_local.destination.z;
    double delta = sqrt(pow(deltax,2)+pow(deltay,2)+pow(deltaz,2));
    bool valid = (now.seconds() - m_time.start > m_time.total) && delta < 0.42;

    // double deltax = std::abs(current.position[0] - m_local.destination.x);
    // double deltay = std::abs(current.position[1] - m_local.destination.y);
    // double deltaz = std::abs(current.position[2] - m_local.destination.z);

    // double tolerance_x = 0.2;
    // double tolerance_y = 0.2;
    // double tolerance_z = 0.2;

    // bool valid_delta = 0;
    // bool valid_time = (now.seconds() - m_time.start > m_time.total);
    // bool valid = valid_time && valid_delta;

    m_states.switchflymode = valid;

    RCLCPP_INFO(rclcpp::get_logger(
        m_log_name), 
        "delta: %f",
        delta
    );
}

bool move_to_gps_target(
    double lat, double lon, double alt,
    px4_msgs::msg::VehicleGlobalPosition sub_gps,
    px4_msgs::msg::TrajectorySetpoint &pub_pose)
{      
    double position_tolerance = 0.5;
    double altitude_tolerance = 0.8;
    double c_lat = sub_gps.lat * 1e-7;
    double c_lon = sub_gps.lon * 1e-7;

    double cur_x, cur_y;
    double tgt_x, tgt_y;
    geo_tool::gps_to_local(
        c_lat, c_lon, 
        c_lat, c_lon, 
        cur_x, cur_y
    );
    geo_tool::gps_to_local(
        c_lat, c_lon, 
        lat, lon, 
        tgt_x, tgt_y
    );
    pub_pose.position[0] = tgt_y;
    pub_pose.position[1] = tgt_x;
    pub_pose.position[2] = -(lat - sub_gps.alt);

    double dx = tgt_x - cur_x;
    double dy = tgt_y - cur_y;
    double dz = std::abs(alt - sub_gps.alt);
    double dist_xy = std::sqrt(dx * dx + dy * dy);

    return (dist_xy < position_tolerance && dz < altitude_tolerance);
}

void Movement::justmove(
    px4_msgs::msg::VehicleOdometry sub_pose, 
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    rclcpp::Time instant_time,
    Waypts target,
    double v = 0.5, bool auto_angle = false)
{
    if(m_states.indicator == 0) {
        m_local.start.x = sub_pose.position[0];
        m_local.start.y = sub_pose.position[1];
        m_local.start.z = sub_pose.position[2];
        m_local.destination = target;

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
        
        double distancex = m_local.destination.x - m_local.start.x;
        double distancey = m_local.destination.y - m_local.start.y;
        double distancez = m_local.destination.z - m_local.start.z;
        double distance  = sqrt(pow(distancex,2) + pow(distancey,2) + pow(distancez,2));

        m_time.total = distance / v;
        m_velocity.vx = distancex / m_time.total;
        m_velocity.vy = distancey / m_time.total;
        m_velocity.vz = distancez / m_time.total;
        if (auto_angle){
            m_velocity.dw = atan2(m_velocity.vy, m_velocity.vx);
        }else {
            tf2_tool::EulerAngles angle;
            tf2_tool::get_euler_angles(sub_pose, angle);
            m_velocity.dw = angle.yaw;
        }
     
        m_time.start = instant_time.seconds();
        m_states.indicator = 1;
        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "控制开始"
        );
    }
    switchflymode(instant_time, sub_pose);

    m_velocity.dt = instant_time.seconds() - m_time.start;

    if(!m_states.switchflymode){
        double out_x = m_local.start.x + m_velocity.dt * m_velocity.vx;
        double out_y = m_local.start.y + m_velocity.dt * m_velocity.vy;
        double out_z = m_local.start.z + m_velocity.dt * m_velocity.vz;

        auto limit_to_destination = [](double out, double v, double dest) -> double {
            if ((v > 0 && out > dest) || (v < 0 && out < dest)) {
                return dest;
            }
            return out;
        };

        out_x = limit_to_destination(out_x, m_velocity.vx, m_local.destination.x);
        out_y = limit_to_destination(out_y, m_velocity.vy, m_local.destination.y);
        out_z = limit_to_destination(out_z, m_velocity.vz, m_local.destination.z);
        
        pub_pose.position[0] = out_x;
        pub_pose.position[1] = out_y;
        pub_pose.position[2] = out_z;
        pub_pose.yaw = m_velocity.dw;

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
        m_states.indicator = 0;
    }
}

void Movement::move_by_offset(
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
    justmove(sub_pose, pub_pose, instant_time, end, v, true);
}

void Movement::change_height(
    px4_msgs::msg::VehicleOdometry sub_pose, 
    px4_msgs::msg::TrajectorySetpoint &pub_pose,
    rclcpp::Time instant_time,
    double high,
    double v = 0.5)
{
    Waypts target = {0, 0, -high};

    justmove(sub_pose, pub_pose, instant_time, target, v, false);
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
            if (dist_bottom_valid && dist_bottom < (start_dist_bottom-0.05)){
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
            if (instant_time.seconds() - m_land.start_time >= 3){
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
