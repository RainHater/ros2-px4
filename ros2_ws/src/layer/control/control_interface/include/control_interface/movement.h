#ifndef _MOVEMENT_H
#define _MOVEMENT_H

#include <px4_msgs/msg/detail/trajectory_setpoint__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "utilities/topic_tool.hpp"

typedef struct {
    double x;
    double y;
    double z;
    double ow;
    double ox;
    double oy;
    double oz;
}UAVpose;

typedef struct {
    double x;
    double y;
    double z;
}Waypts;

class Movement{
public:
    Movement();
    bool switchflymode() const;
protected:
    void switchflymode(rclcpp::Time now, UAVpose currentinfo);
    void justmove(UAVpose currentinfo, 
        px4_msgs::msg::TrajectorySetpoint &pose,
        rclcpp::Time instant_time,
        Waypts start, Waypts end,
        double v, double angle
    );
private:
    struct PubInfo{
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint;
    };

    struct SubInfo{
        TopicListener<px4_msgs::msg::VehicleOdometry> vehicle_odometry;
    };

    struct TimeInfo{
        double start;
        double total;
    };

    struct VelocityProfile{
        double dv = 0.5;        // m_dv
        double dw = 0.25;       // m_dw
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        double dt = 0.0;
    };

    struct StatusBits{
        bool switchflymode;
        int indicator;
    };

private:
    PubInfo m_pub;
    SubInfo m_sub;
    StatusBits m_states;
    TimeInfo m_time;
    VelocityProfile m_velocity;
};

#endif
