#ifndef _TRACK_H
#define _TRACK_H

#include <px4_msgs/msg/detail/sensor_combined__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_attitude__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include "identify/msg/yolo_detections.hpp"

#include "utilities/tf2_tool.hpp"

#include "control_interface/pid.h"


namespace track {
struct NormalTrack{
    identify::msg::YoloDetections detections;
    px4_msgs::msg::VehicleOdometry sub_pose;
    px4_msgs::msg::VehicleAttitude sub_attitude;
    px4_msgs::msg::SensorCombined sensor_combined;
    px4_msgs::msg::TrajectorySetpoint* pub_tra;
};
}

class Track{
public:
    Track();

    void normal_track(track::NormalTrack& normal_info);
private:
    struct CameraInfo{
        float hfov;
        float width;
        float height;
    };

    struct PIDInfo{
        PIDController deviation;
        float last_time;
    };

    struct YamlInfo{
        float deviation_kp = 0.003f;
        float deviation_ki = 0.00001f;
        float deviation_kd = 0.0f;
    };
private:
    rclcpp::Logger m_log;
    PIDInfo m_pid;
    YamlInfo m_yaml;
    CameraInfo m_camera;
};

#endif
