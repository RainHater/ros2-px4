#include "control_interface/movement.h"
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

void Movement::justmove(
    px4_msgs::msg::VehicleOdometry current, 
    px4_msgs::msg::TrajectorySetpoint &pose,
    rclcpp::Time instant_time,
    Waypts target,
    double v = 0.5, bool auto_angle = false)
{
    if(m_states.indicator == 0) { 
        m_local.start.x = current.position[0];
        m_local.start.y = current.position[1];
        m_local.start.z = current.position[2];

        m_local.destination = target;
   
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
            tf2_tool::get_euler_angles(current, angle);
            m_velocity.dw = angle.yaw;
        }
     
        m_time.start = instant_time.seconds();
        m_states.indicator = 1;
        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "控制开始"
        );
    }
    switchflymode(instant_time, current);

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
        
        pose.position[0] = out_x;
        pose.position[1] = out_y;
        pose.position[2] = out_z;
        pose.yaw = m_velocity.dw;

        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "position[0]: %f, position[1]: %f, position[2]: %f",
            pose.position[0],
            pose.position[1],
            pose.position[2]
        );

        RCLCPP_INFO(rclcpp::get_logger(
            m_log_name), 
            "current[0]: %f, current[1]: %f, current[2]: %f",
            current.position[0],
            current.position[1],
            current.position[2]
        );
    }else {
        m_states.indicator = 0;
    }
}

void Movement::move_by_offset(
    px4_msgs::msg::VehicleOdometry current, 
    px4_msgs::msg::TrajectorySetpoint &pose,
    rclcpp::Time instant_time,
    Offset target,
    double v = 0.5, double angle = 0.0)
{
    tf2_tool::EulerAngles angles;
    tf2_tool::get_euler_angles(current, angles);
    float yaw = geo_tool::normalize_angle(angles.yaw + geo_tool::deg2rad(angle));
    float dx = target.forward * std::cos(yaw) - target.right * std::sin(yaw);
    float dy = target.forward * std::sin(yaw) + target.right * std::cos(yaw);
    float dz = -target.up;

    Waypts end;
    end.x = (current.position[0] + dx);
    end.y = (current.position[1] + dy);
    end.z = (current.position[2] + dz);
    justmove(current, pose, instant_time, end, v, true);
}

void Movement::change_height(
    px4_msgs::msg::VehicleOdometry current, 
    px4_msgs::msg::TrajectorySetpoint &pose,
    rclcpp::Time instant_time,
    double high,
    double v = 0.5)
{
    Waypts target = {0, 0, -high};

    justmove(current, pose, instant_time, target, v, false);
}

bool Movement::land_mode(
    double v,
    ModeControl mode_control,
    rclcpp::Time instant_time,
    common_msgs::msg::ArmOffboardStatus px4_mode,
    px4_msgs::msg::VehicleOdometry current_pose,
    px4_msgs::msg::TrajectorySetpoint &pose,
    common_msgs::msg::ArmOffboardStatus &px4_mode_pub,
    TopicListener<px4_msgs::msg::VehicleLocalPosition> local_position)
{   
    bool finish = false;

    switch(m_land.state){
        case 0:{
            m_land.start_state = px4_mode;
            m_land.state = 1;
            break;
        }
        case 1:{
            mode_control.unlock(
                ARM_ENABLE,
                (POSITION | VELOCITY),
                px4_mode, px4_mode_pub
            );
            if (mode_control.wait_busy()){
                tf2_tool::EulerAngles angle;
                tf2_tool::get_euler_angles(current_pose, angle);
                m_land.start_position.x = current_pose.position[0];
                m_land.start_position.y = current_pose.position[1];
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

            pose.position[0] = m_land.start_position.x;
            pose.position[1] = m_land.start_position.y;
            pose.position[2] = NAN;
            pose.velocity[0] = NAN;
            pose.velocity[1] = NAN;
            pose.velocity[2] = v;
            pose.yaw = m_land.dw;
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
                px4_mode_pub.offboard = m_land.start_state.offboard;
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
