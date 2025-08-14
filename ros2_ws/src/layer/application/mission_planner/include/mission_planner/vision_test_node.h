#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <px4_msgs/msg/detail/manual_control_setpoint__struct.hpp>
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

class VisionTestNode : public rclcpp::Node{
public:
    VisionTestNode();
    void initialize();
protected:
    void init_pub();
    void init_sub();
protected:
    void task_loop();
    void calculate_yaw(
        int cx, 
        int image_width, 
        float fov_deg,
        px4_msgs::msg::VehicleOdometry current,
        px4_msgs::msg::TrajectorySetpoint &pose
    );
private:
    enum FlyStep{
        IDLE,
        RISE,
        SWITCH_MODE,
        Hover,
        LAND,
        END,
    };

    struct PubInfo{
        rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr offboard_mode;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint;
    };

    struct SubInfo{
        TopicListener<common_msgs::msg::ArmOffboardStatus> offboard_mode;
        TopicListener<px4_msgs::msg::VehicleOdometry> vehicle_odometry;
        TopicListener<px4_msgs::msg::VehicleLocalPosition> local_position;
        TopicListener<px4_msgs::msg::VehicleAttitude> vehicle_attitude;
        TopicListener<px4_msgs::msg::ManualControlSetpoint> manual_control_setpoint;
        TopicListener<identify::msg::YoloDetections> yolo_detections;
    };

    struct PubMsgInfo{
        common_msgs::msg::ArmOffboardStatus offboard_mode;
        px4_msgs::msg::TrajectorySetpoint trajectory_setpoint;
    };

    struct InterfaceInfo{
        ModeControl mode_control;
        Movement movement; 
        Track track;
        RcSignal rc_signal;
    };
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    FlyStep m_fly;
    PubInfo m_pub;
    SubInfo m_sub;
    PubMsgInfo m_pub_msgs;
    InterfaceInfo m_interface;
    std::string m_yaml_path;
};

#endif
