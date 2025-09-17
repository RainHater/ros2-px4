#ifndef _TOPIC_INFO_HPP
#define _TOPIC_INFO_HPP

#include <string>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>

namespace utilities{
class TopicInfo{
public:
    struct TopicOutInfo{
        std::string PX4_MODE = "/control/out/offboard_mode";
        std::string TRACKING_FEEDBACK = "/vision_pipeline/tracking_feedback";
        std::string YOLO_DETECTIONS = "/yolo_detections";
    };

    struct TopicInInfo{
        std::string PX4_MODE = "/control/in/offboard_mode";
    };

    struct TopicPx4InInfo{
        std::string VEHICLE_COMMAND = "/fmu/in/vehicle_command";
        std::string OFFBOARD_CONTROL_MODE = "/fmu/in/offboard_control_mode";
        std::string PX4_TRAJECTORY_SETPOINT = "/fmu/in/trajectory_setpoint";
        std::string MISSION = "/fmu/in/mission";
        std::string NAVIGATOR_MISSION_ITEM = "/fmu/in/navigator_mission_item";
    };

    struct TopicPx4OutInfo{
        std::string VEHICLE_STATUS = "/fmu/out/vehicle_status_v1";
        std::string VEHICLE_LOCAL_POSITION = "/fmu/out/vehicle_local_position";
        std::string VEHICLE_ODOMETRY = "/fmu/out/vehicle_odometry";
        std::string VEHICLE_GLOBAL_POSITION = "/fmu/out/vehicle_global_position";
        std::string BATTERY_STATUS = "/fmu/out/battery_status";
        std::string VEHICLE_ATTITUDE = "/fmu/out/vehicle_attitude";
        std::string VEHICLE_ATTITUDE_V = "/fmu/out/vehicle_angular_velocity";
        std::string SENSOR_COMBINED = "/fmu/out/sensor_combined";
        std::string MANUAL_CONTROL_SETPOINT = "/fmu/out/manual_control_setpoint";
    };

    //获取单例实例
    static TopicInfo& getInstance() {
        static TopicInfo instance;
        return instance;
    }

    //删除拷贝和赋值
    TopicInfo(const TopicInfo&) = delete;
    TopicInfo& operator=(const TopicInfo&) = delete;

    //Getter
    const TopicOutInfo& topic_out() const { return m_topic_out; }
    const TopicInInfo& topic_in() const { return m_topic_in; }
    const TopicPx4InInfo& topic_px4_in() const { return m_topic_px4_in; }
    const TopicPx4OutInfo& topic_px4_out() const { return m_topic_px4_out; }

private:
    TopicInfo() {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("utilities") + "/config/topic.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        YAML::Node topic_out = config["topic_out"];
        YAML::Node topic_in = config["topic_in"];
        YAML::Node topic_px4_in = config["topic_px4_in"];
        YAML::Node topic_px4_out = config["topic_px4_out"];

        if(topic_out) {
            m_topic_out.PX4_MODE = topic_out["PX4_MODE"].as<std::string>(m_topic_out.PX4_MODE);
            m_topic_out.TRACKING_FEEDBACK = topic_out["TRACKING_FEEDBACK"].as<std::string>(m_topic_out.TRACKING_FEEDBACK);
            m_topic_out.YOLO_DETECTIONS = topic_out["YOLO_DETECTIONS"].as<std::string>(m_topic_out.YOLO_DETECTIONS);
        }

        if(topic_in) {
            m_topic_in.PX4_MODE = topic_in["PX4_MODE"].as<std::string>(m_topic_in.PX4_MODE);
        }

        if(topic_px4_in) {
            m_topic_px4_in.VEHICLE_COMMAND = topic_px4_in["VEHICLE_COMMAND"].as<std::string>(m_topic_px4_in.VEHICLE_COMMAND);
            m_topic_px4_in.OFFBOARD_CONTROL_MODE = topic_px4_in["OFFBOARD_CONTROL_MODE"].as<std::string>(m_topic_px4_in.OFFBOARD_CONTROL_MODE);
            m_topic_px4_in.PX4_TRAJECTORY_SETPOINT = topic_px4_in["PX4_TRAJECTORY_SETPOINT"].as<std::string>(m_topic_px4_in.PX4_TRAJECTORY_SETPOINT);
            m_topic_px4_in.MISSION = topic_px4_in["MISSION"].as<std::string>(m_topic_px4_in.MISSION);
            m_topic_px4_in.NAVIGATOR_MISSION_ITEM = topic_px4_in["NAVIGATOR_MISSION_ITEM"].as<std::string>(m_topic_px4_in.NAVIGATOR_MISSION_ITEM);
        }

        if(topic_px4_out) {
            m_topic_px4_out.VEHICLE_STATUS = topic_px4_out["VEHICLE_STATUS"].as<std::string>(m_topic_px4_out.VEHICLE_STATUS);
            m_topic_px4_out.VEHICLE_LOCAL_POSITION = topic_px4_out["VEHICLE_LOCAL_POSITION"].as<std::string>(m_topic_px4_out.VEHICLE_LOCAL_POSITION);
            m_topic_px4_out.VEHICLE_ODOMETRY = topic_px4_out["VEHICLE_ODOMETRY"].as<std::string>(m_topic_px4_out.VEHICLE_ODOMETRY);
            m_topic_px4_out.VEHICLE_GLOBAL_POSITION = topic_px4_out["VEHICLE_GLOBAL_POSITION"].as<std::string>(m_topic_px4_out.VEHICLE_GLOBAL_POSITION);
            m_topic_px4_out.BATTERY_STATUS = topic_px4_out["BATTERY_STATUS"].as<std::string>(m_topic_px4_out.BATTERY_STATUS);
            m_topic_px4_out.VEHICLE_ATTITUDE = topic_px4_out["VEHICLE_ATTITUDE"].as<std::string>(m_topic_px4_out.VEHICLE_ATTITUDE);
            m_topic_px4_out.VEHICLE_ATTITUDE_V = topic_px4_out["VEHICLE_ATTITUDE_V"].as<std::string>(m_topic_px4_out.VEHICLE_ATTITUDE_V);
            m_topic_px4_out.SENSOR_COMBINED = topic_px4_out["SENSOR_COMBINED"].as<std::string>(m_topic_px4_out.SENSOR_COMBINED);
            m_topic_px4_out.MANUAL_CONTROL_SETPOINT = topic_px4_out["MANUAL_CONTROL_SETPOINT"].as<std::string>(m_topic_px4_out.MANUAL_CONTROL_SETPOINT);
        }
    }

    TopicOutInfo m_topic_out;
    TopicInInfo m_topic_in;
    TopicPx4InInfo m_topic_px4_in;
    TopicPx4OutInfo m_topic_px4_out;
};
}

#endif
