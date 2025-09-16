#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <array>
#include <identify/msg/detail/yolo_detections__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_global_position__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "identify/msg/yolo_detections.hpp"

#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"
#include "utilities/geo_tool.hpp"

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
    void manualControlSetpointCallback(const std::shared_ptr<px4_msgs::msg::ManualControlSetpoint> msg);
    void offboardModeCallback(const std::shared_ptr<common_msgs::msg::ArmOffboardStatus> msg);
    void globalPosCallback(const std::shared_ptr<px4_msgs::msg::VehicleGlobalPosition> msg);
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
        rclcpp::Publisher<common_msgs::msg::ArmOffboardStatus>::SharedPtr offb_mode;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj;
    };

    struct SubInfo{
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom;
        rclcpp::Subscription<identify::msg::YoloDetections>::SharedPtr yolo_dets;
        rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_ctrl_sp;
        rclcpp::Subscription<common_msgs::msg::ArmOffboardStatus>::SharedPtr offb_mode;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos;
    };

    struct PubMsgInfo{
        common_msgs::msg::ArmOffboardStatus offb_mode;
        px4_msgs::msg::TrajectorySetpoint traj;
    };

    struct InterfaceInfo{
        ModeControl mode_ctrl;
        track::Track track;
        movement::Movement movement; 
    };

    struct DroneDataInfo{
        std::array<float, 3> cur_pos;
        std::array<float, 4> flo_q;
        //多个检测目标
        std::vector<identify::msg::YoloDetection> det_targets;
        //当前经纬度
        geo_tool::GeoCoordinate cur_gps;
        //当前offboard模式
        uint16_t  cur_offb;
        //当前arm状态
        uint8_t cur_arm;
        //当前坐标系
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
    YamlInfo m_yaml;
    PubMsgInfo m_pub_msgs;
    //模块接口
    InterfaceInfo m_iface;
    //飞控自身数据
    DroneDataInfo m_drone_data;
};

#endif
