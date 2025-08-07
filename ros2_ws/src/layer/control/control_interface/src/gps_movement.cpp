#include "control_interface/gps_movement.h"
#include <cmath>

constexpr auto WAYPOINT = common_msgs::msg::ArmOffboardStatus::WAYPOINT;
constexpr auto TARGET_ERROR = 5;

GPSMovement::GPSMovement(){
    m_log_name = "全局飞行(gps_movement.cpp)";
    m_status.switchflymode = false; 
}

bool GPSMovement::switchflymode(){
    return m_status.switchflymode;
}

void GPSMovement::move_to_gps_target(
    double lat, double lon, float alt,
    VehicleGlobalPosition sub_gps,
    ArmOffboardStatus &pub_mode)
{
    if (m_status.indicator){    
        // double c_lat = sub_gps.lat / 1e7;
        // double c_lon = sub_gps.lon / 1e7;
        // double c_alt = sub_gps.alt / 1e7;
        // double distance = geo_tool::haversine_3d_distance(
        //     c_lat, c_lon, c_alt,
        //     lat, lon, alt
        // );
        // if (distance < TARGET_ERROR){
        //     pub_mode.lat = NAN;
        //     pub_mode.lon = NAN;
        //     pub_mode.alt = NAN;
        //     pub_mode.offboard = m_status.start_offboard;
        //     pub_mode.target_reached = true;
        //     m_status.indicator = 0;
        //     m_status.switchflymode = true;
        //     RCLCPP_INFO(rclcpp::get_logger(
        //     m_log_name), 
        //         "✅ 已到达目标航点"
        //     );
        // }else {
        //     m_status.switchflymode = false;
        // }
    }else {
        m_status.start_offboard = pub_mode.offboard;
        pub_mode.lat = lat;
        pub_mode.lon = lon;
        pub_mode.alt = alt;
        pub_mode.offboard = WAYPOINT;
        pub_mode.target_reached = false;
        m_status.indicator = 1;
    }
}
