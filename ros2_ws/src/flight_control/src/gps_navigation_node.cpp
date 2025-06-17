#include "flight_control/gps_navigation_node.h"

using std::placeholders::_1;

GpsNavigationNode::GpsNavigationNode() : rclcpp::Node("gps_navigation_node") {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    m_target_gps = create_subscription<common_msgs::msg::TargetGps>(
        "/target_gps", 10, 
        std::bind(&GpsNavigationNode::target_gps_callback, this, _1));
    m_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", qos,
        std::bind(&GpsNavigationNode::target_position_callback, this, _1));
    m_target_position_pub = create_publisher<common_msgs::msg::PositionSetpoint>(
        "/target_position", 10);
}

void GpsNavigationNode::target_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg){
    m_current_gps.latitude = msg->lat;
    m_current_gps.longitude = msg->lon;
    m_current_gps.altitude = msg->alt;
    // RCLCPP_INFO(get_logger(), "current: {lat: %.7f, lon: %.7f}", m_current_gps->lat, m_current_gps->lon);
}

void GpsNavigationNode::target_gps_callback(const common_msgs::msg::TargetGps::SharedPtr msg){
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    RCLCPP_INFO(this->get_logger(), "Target_latitude: %.7f, Target_Longitude: %.7f", latitude, longitude);  
    convert_gps_to_position(msg);  
}

void GpsNavigationNode::convert_gps_to_position(const common_msgs::msg::TargetGps::SharedPtr msg){

    double dlat = (msg->latitude - m_current_gps.latitude) * m_deg_to_m;
    double dlon = (msg->longitude - m_current_gps.longitude) * m_deg_to_m * std::cos(m_current_gps.latitude * M_PI / 180.0);
    double dz = msg->altitude - m_current_gps.altitude;

    publish_target_position(dlat, dlon, dz);
}

void GpsNavigationNode::publish_target_position(double dlat, double dlon, double dz){
    common_msgs::msg::PositionSetpoint msg;
    msg.x = dlat;
    msg.y = dlon;
    msg.z = dz;

    RCLCPP_INFO(this->get_logger(), "Publishing goal pose: [%.2f, %.2f, %.2f]",
                    msg.x, msg.y, msg.z);
    m_target_position_pub->publish(msg);
}

int main(int argc, char *argv[]) {
    std::cout << "Starting GpsNavigationNode follower node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNavigationNode>());

    rclcpp::shutdown();
    return 0;
}
