#ifndef _UTILITES_H
#define _UTILITES_H

#include <rclcpp/rclcpp.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <string>
#include <math.h>
#include <array>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace utilities{
namespace convert{
    constexpr double EARTH_RADIUS_M = 6371000.0;

        struct GeoCoord  {
        double  lat;     // 纬度
        double  lon;     // 经度
        float   alt;     // 高度
    };

    //角度转弧度
    double deg2rad(double deg);

    //GPS → ENU 简易转换函数（参考点 + 当前点 → x, y）
    void gps_to_local(
        double lat0, double lon0, 
        double lat, double lon, 
        float &x, float &y
    );

    double haversine_distance(
        double lat1_deg, double lon1_deg, 
        double lat2_deg, double lon2_deg
    );

    float normalize_angle(float angle);

    double flo_to_yaw(std::array<float, 4> flo_q);
}
}

#endif
