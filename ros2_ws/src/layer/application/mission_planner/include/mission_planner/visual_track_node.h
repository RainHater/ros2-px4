#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <array>
#include <identify/msg/detail/yolo_detections__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "identify/msg/yolo_detections.hpp"

#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"

#include "control_interface/mode_control.h"
#include "control_interface/movement.h"
#include "control_interface/track.h"
#include "control_interface/rc_signal.h"

class VisualTrack : public rclcpp::Node{
public:
    VisualTrack();
    void initialize();
protected:
    void initPub();
    void initSub();
protected:
    void taskLoop();

    //消息回调函数
    void vehicleOdometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg);
    void yoloDetectionsCallback(const std::shared_ptr<identify::msg::YoloDetections> msg);
private:
    enum FlyStep{
        IDLE,
        RISE,
        SWITCH_MODE,
        WAIT,
        Hover,
        LAND,
    };

    struct PubInfo{
        rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr offboard_mode;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint;
    };

    struct SubInfo{
        TopicListener<common_msgs::msg::ArmOffboardStatus> offboard_mode;
        TopicListener<px4_msgs::msg::VehicleLocalPosition> local_position;
        TopicListener<px4_msgs::msg::ManualControlSetpoint> manual_control_setpoint;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry;
        rclcpp::Subscription<identify::msg::YoloDetections>::SharedPtr yolo_detections;
    };

    struct PubMsgInfo{
        common_msgs::msg::ArmOffboardStatus offboard_mode;
        px4_msgs::msg::TrajectorySetpoint trajectory_setpoint;
    };

    struct InterfaceInfo{
        ModeControl mode_control;
        RcSignal rc_signal;
        track::Track track;
        movement::Movement movement; 
    };

    struct DroneDataInfo{
        std::array<float, 3> cur_pos;
        std::array<float, 4> flo_q;
        //多个检测目标
        std::vector<identify::msg::YoloDetection> det_targets;
        uint8_t cur_coor;
        //检测是否有效
        bool is_target_valid;
        //检测是否变化
        bool is_detection_changed;
    };

    struct YamlInfo{
        float lift_height = 1.5f;
        bool outdoor_flag = false;
        bool switch_mode = false;
        int track_mode = 0;
    };
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    FlyStep m_fly;
    PubInfo m_pub;
    SubInfo m_sub;
    PubMsgInfo m_pub_msgs;
    InterfaceInfo m_interface;
    YamlInfo m_yaml;
    DroneDataInfo m_drone_data;
};

#endif
