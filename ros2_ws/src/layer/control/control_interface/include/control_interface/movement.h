#ifndef _MOVEMENT_H
#define _MOVEMENT_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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
    bool wait_busy() const;
    void justmove(
        px4_msgs::msg::VehicleOdometry current, 
        px4_msgs::msg::TrajectorySetpoint &pose,
        rclcpp::Time instant_time,
        Waypts start, Waypts end,
        double v, double angle
    );
protected:
    void switchflymode(
        rclcpp::Time now, 
        px4_msgs::msg::VehicleOdometry current
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
        double dw = 0.25;       // m_dw
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        double dt = 0.0;

    };

    struct StatusBits{
        int indicator;
        bool switchflymode;
    };

    struct LocationInfo{
        Waypts start;
        Waypts destination;
    };

private:
    std::string m_log_name;
    StatusBits m_states;
    TimeInfo m_time;
    VelocityProfile m_velocity;
    
    LocationInfo m_local;
};

#endif
