#include "control_interface/movement.h"
#include "utilities/topic_name.hpp"

Movement::Movement()
{
}

bool Movement::switchflymode() const{
    return m_states.switchflymode;
}

void Movement::switchflymode(rclcpp::Time now, UAVpose currentinfo){
    // double deltax = currentinfo.x-local_destination.x;
    // double deltay = currentinfo.y-local_destination.y;
    // double deltaz = currentinfo.z-local_destination.z;
    // double delta  =sqrt(pow(deltax,2)+pow(deltay,2)+pow(deltaz,2));
    bool valid = (now.seconds() - m_time.start > m_time.total) /*&& delta < 0.2*/;
    m_states.switchflymode = valid;
}

void Movement::justmove(UAVpose currentinfo, 
    px4_msgs::msg::TrajectorySetpoint &pose,
    rclcpp::Time instant_time,
    Waypts start, Waypts end,
    double v, double angle)
{
    if(m_states.indicator==0)
    {
        double distancex = end.x - start.x;
        double distancey = end.y - start.y;
        double distancez = end.z - start.z;
        double distance  = sqrt(pow(distancex,2) + pow(distancey,2) + pow(distancez,2));

        m_time.total = distance / m_velocity.dv;
        m_velocity.vx = distancex / m_time.total;
        m_velocity.vy = distancey / m_time.total;
        m_velocity.vz = distancez / m_time.total;

        m_time.start = instant_time.seconds();
        m_states.indicator = 1;
    }
    switchflymode(instant_time, currentinfo);

    m_velocity.dt = instant_time.seconds() - m_time.start;

    if(m_states.switchflymode){
        m_states.indicator = 0;
    }else {
        pose.position[0] = start.x + m_velocity.dt * m_velocity.vx;
        pose.position[1] = start.y + m_velocity.dt * m_velocity.vy;
        pose.position[2] = start.z + m_velocity.dt * m_velocity.vz;
    }
}
