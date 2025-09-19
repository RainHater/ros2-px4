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

    //普通跟踪
    void normalTrack(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    //普通跟踪v1(位置)
    void normalTrack_v1(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    void normalTrack_v2(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    void normalTrack_v3(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    void normalTrack_v4(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 3> g_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    void normalTrack_v5(
        bool is_target_valid,
        int64_t dt,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        std::vector<identify::msg::YoloDetection> det_targets,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    //更新定点值
    void updateLastPostition(std::array<float, 3> pos);
private:
    //相机参数
    struct CameraInfo{
        float width;
        float height;
        float hfov;
    };
    //普通跟踪计算
    struct NormalTrackInfo{
        PIDController yaw;
        PIDController ud;
        PIDController fb;
        std::array<float, 3> last_pos;
        identify::msg::YoloDetection last_detection;
        float thre_area;
        bool last_pos_init = false;
        bool last_pos_update = false;
    };
    //yaml 配置文件
    struct YamlInfo{
        PidInfo yaw = {0.003f, 0.00001f, 0.0f, 1.5708f};
        PidInfo ud = {0.003f, 0.00001f, 0.0f, 0.5f};
        PidInfo fb = {0.003f, 0.00001f, 0.0f, 0.4f};
        float area_th = 0.41f;
        bool is_filter = false;
    };
private:
    rclcpp::Logger m_log;
    NormalTrackInfo m_normal_track;
    YamlInfo m_yaml;
    CameraInfo m_camera;
};
}
#endif
