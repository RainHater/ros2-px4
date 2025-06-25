#include "flight_control/state_estimator_node.h"

using std::placeholders::_1;

StateEstimatorNode::StateEstimatorNode() 
    : rclcpp::Node("state_estimator_node") {
    RCLCPP_INFO(get_logger(), "Starting state_estimator_node follower node...");
    
    m_trajectory_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
    m_target_gps_sub = create_subscription<common_msgs::msg::TargetGps>(
        "/control/target_gps", 10, 
        std::bind(&StateEstimatorNode::target_gps_callback, this, _1));
    m_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/interface/out/vehicle_global_position", 10,
        std::bind(&StateEstimatorNode::current_gps_callback, this, _1)); 
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&StateEstimatorNode::timer_callback, this));
}

void StateEstimatorNode::timer_callback(){
    convert_gps_to_position();
}

void StateEstimatorNode::current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition &msg){
    m_current_gps.lat = msg.lat / 1e7;
    m_current_gps.lon = msg.lon / 1e7;
    double alt_m = msg.alt / 1e3; 
    if (m_home_alt == 0) {
        m_home_alt = alt_m;
    } else {
        m_current_gps.alt = alt_m - m_home_alt;
    }
    m_get_current_gps_finish = true;
}

void StateEstimatorNode::target_gps_callback(const common_msgs::msg::TargetGps &msg){
    m_target_gps = msg;
    m_get_target_gps_finish = true;
}

void StateEstimatorNode::convert_gps_to_position(){
    if (!m_get_target_gps_finish || !m_get_current_gps_finish)
        return;
    m_get_target_gps_finish = false;
    m_get_current_gps_finish = false;

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

    RCLCPP_INFO(get_logger(), "gps_dist: %.7f", dist);
    RCLCPP_INFO(get_logger(), "current: {lat: %.7f, lon: %.7f, alt: %.7f}", lat_c, lon_c, alt_c);
    RCLCPP_INFO(get_logger(), "target: {lat: %.7f, lon: %.7f, alt: %.7f}", lat_t, lon_t, alt_t);
}

void StateEstimatorNode::publish_target_position(double dlat, double dlon, double dz){
    common_msgs::msg::TrajectorySetPoint msg;
    msg.position[0] = dlat;
    msg.position[1] = dlon;
    msg.position[2] = -dz;

    RCLCPP_INFO(this->get_logger(), "Publishing position: [%.2f, %.2f, %.2f]",
                    msg.position[0], 
                    msg.position[1], 
                    msg.position[2]);
    m_trajectory_setpoint_pub->publish(msg);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorNode>());

    rclcpp::shutdown();
    return 0;
}
