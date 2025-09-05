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

#include <builtin_interfaces/msg/time.hpp>

#include "identify/msg/yolo_detections.hpp"

#include "utilities/tf2_tool.hpp"
#include "utilities/logger_tool.hpp"

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
    void update_last_postition(std::array<float, 3> pos);
private:

    struct CameraInfo{
        float hfov;
        float width;
        float height;
    };

    struct PidCalInfo{
        PIDController yaw;
        PIDController ud;
        PIDController fb;
        std::array<float, 3> last_postition;
        double last_time;
        bool last_pos_init = false;
        bool last_pos_update = false;
    };

    struct YamlInfo{
        PidInfo yaw = {0.003f, 0.00001f, 0.0f};
        PidInfo ud = {0.003f, 0.00001f, 0.0f};
        PidInfo fb = {0.003f, 0.00001f, 0.0f};
        float area_th = 0.41f;
    };

    struct NormalTrack{
        float thre_area;
    };
private:
    rclcpp::Logger m_log;
    LoggerTool m_logger;
    PidCalInfo m_pid;
    YamlInfo m_yaml;
    CameraInfo m_camera;
    NormalTrack m_normal_track;
};

#endif
