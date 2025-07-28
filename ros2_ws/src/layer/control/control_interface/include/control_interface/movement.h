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

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"
#include  "utilities/geo_tool.hpp"

#include "control_interface/mode_control.h"

struct UAVpose{
    double x;
    double y;
    double z;
    double ow;
    double ox;
    double oy;
    double oz;
};

struct Waypts{
    double x;
    double y;
    double z;
};

struct Offset{
    double up;
    double forward;
    double right;
};

class Movement{
public:
    Movement();
    bool wait_busy() const;

    //根据局部整体坐标移动
    void justmove(
        px4_msgs::msg::VehicleOdometry current, 
        px4_msgs::msg::TrajectorySetpoint &pose,
        rclcpp::Time instant_time,
        Waypts target,
        double v, bool auto_angle
    );

    //根据当前坐标进行移动
    void move_by_offset(
        px4_msgs::msg::VehicleOdometry current, 
        px4_msgs::msg::TrajectorySetpoint &pose,
        rclcpp::Time instant_time,
        Offset target,
        double v, double angle
    );

    //起飞高度
    void change_height(
        px4_msgs::msg::VehicleOdometry current, 
        px4_msgs::msg::TrajectorySetpoint &pose,
        rclcpp::Time instant_time,
        double high,
        double v
    );

    void land_mode(
        ModeControl mode_control,
        common_msgs::msg::ArmOffboardStatus px4_mode,
        px4_msgs::msg::VehicleOdometry current_pose,
        px4_msgs::msg::VehicleLocalPosition local_position,
        px4_msgs::msg::TrajectorySetpoint &pose,
        common_msgs::msg::ArmOffboardStatus &px4_mode_pub,
        double v
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
        double dw = 0.0;
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

    struct LandInfo{
        common_msgs::msg::ArmOffboardStatus start_state;
        Waypts start_position;
        double dw;
        int state;
    };
private:
    std::string m_log_name;
    StatusBits m_states;
    TimeInfo m_time;
    VelocityProfile m_velocity;
    LocationInfo m_local;
    LandInfo m_land;
};

#endif
