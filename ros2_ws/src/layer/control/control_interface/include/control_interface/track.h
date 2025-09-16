#ifndef _TRACK_H
#define _TRACK_H

#include <identify/msg/detail/yolo_detection__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <builtin_interfaces/msg/time.hpp>

#include "identify/msg/yolo_detections.hpp"

#include "utilities/tf2_tool.hpp"
#include "utilities/type_tool.hpp"
#include "utilities/log_printf_tool.hpp"

#include "control_interface/pid.h"

namespace track {
class Track{
public:
    Track();

    //普通跟踪
    void normal_track(
        identify::msg::YoloDetections detections,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    //普通跟踪v1(位置)
    void normal_track_v1(
        identify::msg::YoloDetections detections,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    void normal_track_v2(
        identify::msg::YoloDetections detections,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    void normal_track_v3(
        identify::msg::YoloDetections detections,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    //更新定点值
    void update_last_postition(std::array<float, 3> pos);
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
        PidInfo yaw = {0.003f, 0.00001f, 0.0f};
        PidInfo ud = {0.003f, 0.00001f, 0.0f};
        PidInfo fb = {0.003f, 0.00001f, 0.0f};
        float area_th = 0.41f;
    };
private:
    rclcpp::Logger m_log;
    NormalTrackInfo m_normal_track;
    YamlInfo m_yaml;
    CameraInfo m_camera;
};
}
#endif
