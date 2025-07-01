#include "perception/state_estimator_node.h"
#include "utilities/util_topic.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

StateEstimatorNode::StateEstimatorNode() 
    : rclcpp::Node("state_estimator_node") {
    RCLCPP_INFO(get_logger(), "Starting state_estimator_node follower node...");
    
    init_subscription();
    init_service();
    
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&StateEstimatorNode::timer_callback, this));
}

void StateEstimatorNode::timer_callback(){

}

void StateEstimatorNode::init_subscription(){
    m_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/interface/out/vehicle_global_position", 10,
        std::bind(&StateEstimatorNode::current_gps_callback, this, _1)); 
    m_current_setpoing_sub = utils::make_simple_subscription<
        px4_msgs::msg::VehicleOdometry>(
        "/interface/out/vehicle_odometry",
        10, this, m_current_setpoint);
}

void StateEstimatorNode::init_service(){
    m_gps_to_local_srv = create_service<TransformGpsToLocal>(
        "/perception/transform_gps_to_local", 
        std::bind(&StateEstimatorNode::handle_gps_to_local, this, _1, _2));
}

void StateEstimatorNode::current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg){
    
    double lat = msg->lat / 1e7;
    double lon = msg->lon / 1e7;
    double alt_m = msg->alt / 1e3; 

    if (!m_reference_initialized && 
        msg->lat_lon_valid &&
        msg->alt_valid){
        m_reference_gps.lat = lat;
        m_reference_gps.lon = lon;
        m_reference_gps.alt = alt_m;
        m_reference_initialized = true;

        RCLCPP_INFO(this->get_logger(), "current_gps_callback: lat=%.7f, lon=%.7f, alt=%.2f",
                    lat, lon, alt_m);
    }
    m_current_gps.lat = lat;
    m_current_gps.lon = lon;
    m_current_gps.alt = alt_m;
}

void StateEstimatorNode::handle_gps_to_local(
        const std::shared_ptr<TransformGpsToLocal::Request> request,
        std::shared_ptr<TransformGpsToLocal::Response> response){

    if (!m_reference_initialized) {
        RCLCPP_WARN(this->get_logger(), "Reference origin not initialized, cannot transform.");
        response->x = 0;
        response->y = 0;
        response->z = 0;
        return;
    }

    double x = 0.0, y = 0.0;
    gps_to_local(m_reference_gps.lat, m_reference_gps.lon,
                 request->latitude, request->longitude, x, y);
    
    std::array<double, 3> target_position = {x, y, m_reference_gps.lat - request->altitude};
    const float HORIZONTAL_DIST_THRESHOLD = 0.9f;
    const float VERTICAL_DIST_THRESHOLD = 0.4f;
    
    float dx = target_position[0] - m_current_setpoint.position[0];
    float dy = target_position[1] - m_current_setpoint.position[1];
    float dz = target_position[2] - m_current_setpoint.position[2];

    float horizontal_dist = std::hypot(dx, dy);
    float vertical_dist = std::abs(dz);
    bool arrive = (horizontal_dist < HORIZONTAL_DIST_THRESHOLD) && (vertical_dist < VERTICAL_DIST_THRESHOLD);
    
    response->x = target_position[0];
    response->y = target_position[1];
    response->z = target_position[2];
    response->lat = m_current_gps.lat;
    response->lon = m_current_gps.lon;
    response->alt = m_current_gps.alt;
    response->arrive = arrive;
    
    // RCLCPP_INFO(get_logger(), "handle_gps_to_local vertical_dist: %f, horizontal_dist: %f", 
    // vertical_dist, horizontal_dist);

    // RCLCPP_INFO(this->get_logger(), "Converted GPS(%.6f, %.6f, %.2f) → ENU(%.2f, %.2f, %.2f)",
    //             request->latitude, request->longitude, request->altitude,
    //             x, y, response->z);
}

double StateEstimatorNode::deg2rad(double deg){
    return deg * M_PI / 180.0;
}

void StateEstimatorNode::gps_to_local(double lat0, double lon0, double lat, double lon, double &x, double &y){

    //简化的球面近似方法（ENU坐标）
    //地球半径，单位米
    double R = 6378137.0; 
    double dLat = deg2rad(lat - lat0);
    double dLon = deg2rad(lon - lon0);

    //东向（x）
    x = R * dLon * cos(deg2rad(lat0)); 
    //北向（y）
    y = R * dLat;                      
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorNode>());

    rclcpp::shutdown();
    return 0;
}
