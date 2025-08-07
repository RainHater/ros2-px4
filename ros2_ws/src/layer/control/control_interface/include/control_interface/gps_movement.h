#ifndef _GPS_MOVEMENT_H
#define _GPS_MOVEMENT_H

#include <common_msgs/msg/detail/arm_offboard_status__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_global_position.hpp>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/geo_tool.hpp"

class GPSMovement{
public:
    using VehicleGlobalPosition = px4_msgs::msg::VehicleGlobalPosition;
    using ArmOffboardStatus = common_msgs::msg::ArmOffboardStatus;
public:
    GPSMovement();
    //等待飞行模式
    bool switchflymode();
    //飞往目标经纬度
    void move_to_gps_target(
        double lat, double lon, float alt,
        VehicleGlobalPosition sub_gps,
        ArmOffboardStatus &pub_mode
    );
protected:
    
private:
    struct StatusBits{
        int indicator;
        uint16_t start_offboard;
        bool switchflymode;
    };
private:
    std::string m_log_name;
    StatusBits m_status;
};

#endif
