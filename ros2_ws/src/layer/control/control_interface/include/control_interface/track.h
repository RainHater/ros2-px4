#ifndef _TRACK_H
#define _TRACK_H

#include <identify/msg/detail/yolo_detection__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <builtin_interfaces/msg/time.hpp>

#include "identify/msg/yolo_detections.hpp"

#include "utilities/utilities.h"
#include "utilities/log_printf_tool.h"

#include "control_interface/pid.h"

namespace track {
class Track{
public:
    Track();

    //普通跟踪-切换速度模式跟踪
    void normalTrack(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
protected:
    void readYaml();
private:
    //普通跟踪计算
    struct NormalTrackInfo{
        PIDController yaw;
        PIDController ud;
        PIDController fb;
    };
    //yaml 配置文件
    struct YamlInfo{
        PidInfo yaw = {0.003f, 0.00001f, 0.0f, 1.5708f};
        PidInfo ud = {0.003f, 0.00001f, 0.0f, 0.5f};
        PidInfo fb = {0.003f, 0.00001f, 0.0f, 0.4f};
        float area_th = 0.41f;
    };
private:
    rclcpp::Logger m_log;
    NormalTrackInfo m_normal_track;
    YamlInfo m_yaml;
};
}
#endif
