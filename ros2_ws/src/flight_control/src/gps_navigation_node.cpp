#include "flight_control/gps_navigation_node.h"
#include <rclcpp/node.hpp>

using std::placeholders::_1;

GpsNavigationNode::GpsNavigationNode() : rclcpp::Node("gps_navigation_node") {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    m_global_position_subscription = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            std::bind(&GpsNavigationNode::global_position_callback, this, _1));
}

void GpsNavigationNode::global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg){

    double latitude = msg->lat;
    double longitude = msg->lon;
    RCLCPP_INFO(this->get_logger(), "Latitude: %.7f, Longitude: %.7f", latitude, longitude);    
}

int main(int argc, char *argv[]) {
    std::cout << "Starting GpsNavigationNode follower node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNavigationNode>());

    rclcpp::shutdown();
    return 0;
}
