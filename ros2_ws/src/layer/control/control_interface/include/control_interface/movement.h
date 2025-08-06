#ifndef _MOVEMENT_H
#define _MOVEMENT_H

#include <px4_msgs/msg/detail/vehicle_global_position__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"
#include "utilities/geo_tool.hpp"

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

    //飞往目标经纬度
    bool move_to_gps_target(
        double lat, double lon, double alt,
        px4_msgs::msg::VehicleGlobalPosition sub_gps,
        px4_msgs::msg::TrajectorySetpoint &pub_pose
    );

    //根据局部整体坐标移动
    void justmove(
        px4_msgs::msg::VehicleOdometry sub_pose, 
        px4_msgs::msg::TrajectorySetpoint &pub_pose,
        rclcpp::Time instant_time,
        Waypts target,
        double v, bool auto_angle
    );

    //根据当前坐标进行移动
    void move_by_offset(
        px4_msgs::msg::VehicleOdometry sub_pose, 
        px4_msgs::msg::TrajectorySetpoint &pub_pose,
        rclcpp::Time instant_time,
        Offset target,
        double v, double angle
    );

    //起飞高度
    void change_height(
        px4_msgs::msg::VehicleOdometry sub_pose, 
        px4_msgs::msg::TrajectorySetpoint &pub_pose,
        rclcpp::Time instant_time,
        double high,
        double v
    );

    bool land_mode(
        double v,
        ModeControl mode_control,
        rclcpp::Time instant_time,
        common_msgs::msg::ArmOffboardStatus sub_px4_mode,
        px4_msgs::msg::VehicleOdometry sub_pose,
        px4_msgs::msg::TrajectorySetpoint &pub_pose,
        common_msgs::msg::ArmOffboardStatus &pub_px4_mode,
        TopicListener<px4_msgs::msg::VehicleLocalPosition> local_position
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
        uint64_t start_time;
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
