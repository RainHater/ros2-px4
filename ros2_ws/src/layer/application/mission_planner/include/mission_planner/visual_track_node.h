#ifndef _SIMPLE_INDOOR_CONTROL_TASK_H
#define _SIMPLE_INDOOR_CONTROL_TASK_H

#include <px4_msgs/msg/detail/vehicle_local_position__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "identify/msg/yolo_detections.hpp"

#include "utilities/topic_name.h"
#include "utilities/utilities.h"

#include "control_interface/mode_control.h"
#include "control_interface/movement.h"
#include "control_interface/track.h"

class VisualTrack : public rclcpp::Node{
public:
    VisualTrack();
    void initialize();
protected:
    void initPub();
    void initSub();
    void initYaml();
protected:
    void taskLoop();
    void pushMsgs();
    void getCurCoor();
    int64_t getCurMs();
    bool readVisionChange();

    //消息回调函数
    void vehicleOdometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg);
    void yoloDetectionsCallback(const std::shared_ptr<identify::msg::YoloDetections> msg);
    void manualControlSetpointCallback(const std::shared_ptr<px4_msgs::msg::ManualControlSetpoint> msg);
    void offboardModeCallback(const std::shared_ptr<common_msgs::msg::ArmOffboardStatus> msg);
    void globalPosCallback(const std::shared_ptr<px4_msgs::msg::VehicleGlobalPosition> msg);
    void locPosCallback(const std::shared_ptr<px4_msgs::msg::VehicleLocalPosition> msg);
private:
    enum FlyStep{
        IDLE,
        UNLOCK,
        RISE,
        WAIT_10S,
        SWITCH_MODE,
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
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr loc_pos;
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
        std::array<float, 3> loc_pos;
        //计算位置
        std::array<float, 4> flo_q;
        //多个检测目标
        std::vector<identify::msg::YoloDetection> det_targets;
        int64_t detect_last_dt;
        //检测dt
        int64_t detect_dt;
        //当前经纬度
        utilities::convert::GeoCoord cur_gps;
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
        int dt = 0;
        int track_mode = 0;
        int test_mode = 0;
        bool outdoor_flag = false;
        bool switch_mode = false;
        bool is_loc_pos = false;
        bool is_wait_vision = true;
        bool is_indoor_sim = false;
        std::array<bool, 3> is_enable_dir = {true, true, true};
        bool is_track = true;
        bool is_wait_10s = true;
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
    uint64_t m_wait_last_time = 0;
};

#endif
