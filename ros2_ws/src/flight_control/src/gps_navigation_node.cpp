#include "flight_control/gps_navigation_node.h"
#include <rclcpp/logging.hpp>

using std::placeholders::_1;

GpsNavigationNode::GpsNavigationNode() : rclcpp::Node("gps_navigation_node") {
    RCLCPP_INFO(get_logger(), "Starting gps_navigation_node follower node...");

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_target_gps_sub = create_subscription<common_msgs::msg::TargetGps>(
        "/target_gps", 10, 
        std::bind(&GpsNavigationNode::target_gps_callback, this, _1));
    m_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", qos,
        std::bind(&GpsNavigationNode::current_gps_callback, this, _1));
    m_target_position_pub = create_publisher<common_msgs::msg::PositionSetpoint>(
        "/target_position", 10);
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&GpsNavigationNode::timer_callback, this));
}

void GpsNavigationNode::current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg){
    m_current_gps.lat = msg->lat / 1e7;
    m_current_gps.lon = msg->lon / 1e7;
    double alt_m = msg->alt / 1e3; 
    if (m_home_alt == 0) {
        m_home_alt = alt_m;
    } else {
        m_current_gps.alt = alt_m - m_home_alt;
    }
}

void GpsNavigationNode::target_gps_callback(const common_msgs::msg::TargetGps::SharedPtr msg){
    m_target_gps = *msg;
    m_convert_flag = true;
}

void GpsNavigationNode::timer_callback(){

    convert_gps_to_position();
}

void GpsNavigationNode::convert_gps_to_position(){

    if (!m_convert_flag)
        return;
    m_convert_flag = false;

    double lat_t = m_target_gps.lat;
    double lon_t = m_target_gps.lon;
    double alt_t = m_target_gps.alt;
    double lat_c = m_current_gps.lat;
    double lon_c = m_current_gps.lon;
    double alt_c = m_current_gps.alt;

    double dlat_rad = (lat_t - lat_c) * M_PI/180.0;
    double dlon_rad = (lon_t - lon_c) * M_PI/180.0;
    double north = m_R * dlat_rad;
    double east  = m_R * cos(lat_c * M_PI/180.0) * dlon_rad;
    double dz = alt_t - alt_c;

    double dist = std::hypot(north, east);
    if (dist < 0.006){
        publish_target_position(0, 0, 0);
    }else {
        publish_target_position(north, east, dz);
    } 

    RCLCPP_INFO(get_logger(), "gps dist: %.7f", dist);
    RCLCPP_INFO(get_logger(), "current: {lat: %.7f, lon: %.7f, alt: %.7f}", lat_c, lon_c, alt_c);
    RCLCPP_INFO(get_logger(), "target: {lat: %.7f, lon: %.7f, alt: %.7f}", lat_t, lon_t, alt_t);
}

void GpsNavigationNode::publish_target_position(double dlat, double dlon, double dz){
    common_msgs::msg::PositionSetpoint msg;
    msg.x = dlat;
    msg.y = dlon;
    msg.z = -dz;

    RCLCPP_INFO(this->get_logger(), "Publishing goal pose: [%.2f, %.2f, %.2f]",
                    msg.x, msg.y, msg.z);
    m_target_position_pub->publish(msg);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNavigationNode>());

    rclcpp::shutdown();
    return 0;
}
