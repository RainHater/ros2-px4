#include "control_interface/movement.h"
#include "utilities/topic_name.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

Movement::Movement()
{
    m_log_name = "控制无人机(movement.cpp)";
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
    bool valid = (now.seconds() - m_time.start > m_time.total) && delta < 0.2;
    
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
    Waypts start, Waypts end,
    double v = 0.5, double angle = 0.25)
{
    if(m_states.indicator == 0)
    {       
        m_local.start.x = current.position[0];
        m_local.start.y = current.position[1];
        m_local.start.z = current.position[2];

        double distancex = end.x - m_local.start.x;
        double distancey = end.y - m_local.start.y;
        double distancez = end.z - m_local.start.z;
        double distance  = sqrt(pow(distancex,2) + pow(distancey,2) + pow(distancez,2));
        (void)angle;

        m_time.total = distance / v;
        m_velocity.vx = distancex / m_time.total;
        m_velocity.vy = distancey / m_time.total;
        m_velocity.vz = distancez / m_time.total;

        m_local.destination = end;
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
