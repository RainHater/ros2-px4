#include "utilities/utilities.h"

namespace utilities{
namespace log_printf{
void printf_log_pos(
    rclcpp::Logger log,
    std::array<float, 3> target_pos, 
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "pub_pos[0]: %f, pub_pos[1]: %f, pub_pos[2]: %f, "
        "cur_pos[0]: %f, cur_pos[1]: %f, cur_pos[2]: %f",
        target_pos[0],
        target_pos[1],
        target_pos[2],
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}

void printf_log_cur_pos(
    rclcpp::Logger log,
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "cur_pos[0]: %f, cur_pos[1]: %f, cur_pos[2]: %f",
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}

void printf_log_title_pos(
    rclcpp::Logger log,
    std::string title, 
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "%s: pos[0]: %f, pos[1]: %f, pos[2]: %f",
        title.c_str(), 
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}
}

namespace convert{
double deg2rad(double deg){
    return deg * M_PI / 180.0;
}

void gps_to_local(double lat0, double lon0, 
    double lat, double lon, 
    float &x, float &y)
{
    //简化的球面近似方法（ENU坐标）
    double dLat = deg2rad(lat - lat0);
    double dLon = deg2rad(lon - lon0);

    //东向（x）
    x = EARTH_RADIUS_M * dLon * cos(deg2rad(lat0)); 
    //北向（y）
    y = EARTH_RADIUS_M * dLat;     
}

double haversine_distance(
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

float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

double flo_to_yaw(std::array<float, 4> flo_q){
    auto eigen_q = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(flo_q);
    auto cur_yaw = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(eigen_q);

    return cur_yaw;
}
}
}
