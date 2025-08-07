#pragma once

#include <math.h>

namespace geo_tool {
struct GeoCoordinate {
    double lat;     // 纬度
    double lon;     // 经度
    float alt;     // 高度
};

constexpr double EARTH_RADIUS_M = 6371000.0;

//角度转弧度
inline double deg2rad(double deg){
    return deg * M_PI / 180.0;
}
//GPS → ENU 简易转换函数（参考点 + 当前点 → x, y）
inline void gps_to_local(double lat0, double lon0, 
    double lat, double lon, 
    double &x, double &y)
{
    //简化的球面近似方法（ENU坐标）
    double dLat = deg2rad(lat - lat0);
    double dLon = deg2rad(lon - lon0);

    //东向（x）
    x = EARTH_RADIUS_M * dLon * cos(deg2rad(lat0)); 
    //北向（y）
    y = EARTH_RADIUS_M * dLat;     
}

inline double haversine_distance(
    double lat1_deg, double lon1_deg, 
    double lat2_deg, double lon2_deg)
{
    double lat1_rad = deg2rad(lat1_deg);
    double lon1_rad = deg2rad(lon1_deg);
    double lat2_rad = deg2rad(lat2_deg);
    double lon2_rad = deg2rad(lon2_deg);

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon / 2) * std::sin(dlon / 2);

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return EARTH_RADIUS_M * c;
}

inline double haversine_3d_distance(
    double lat1_deg, double lon1_deg, double alt1_m,
    double lat2_deg, double lon2_deg, double alt2_m)
{
    constexpr double R = 6371000.0; // 地球平均半径 [m]
    const double deg_to_rad = M_PI / 180.0;

    // 将纬度、经度转换为弧度
    double lat1 = lat1_deg * deg_to_rad;
    double lon1 = lon1_deg * deg_to_rad;
    double lat2 = lat2_deg * deg_to_rad;
    double lon2 = lon2_deg * deg_to_rad;

    // 经纬度差值
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Haversine formula 计算地表 2D 距离
    double a = std::pow(std::sin(dlat / 2), 2) +
               std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double surface_distance = R * c;

    // 高度差
    double dalt = alt2_m - alt1_m;

    // 3D 空间距离
    return std::sqrt(surface_distance * surface_distance + dalt * dalt);
}

inline float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
}
