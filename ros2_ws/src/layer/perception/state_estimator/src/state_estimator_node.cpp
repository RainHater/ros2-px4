#include "state_estimator/state_estimator_node.h"
#include "utilities/topic_name.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

StateEstimatorNode::StateEstimatorNode() 
    : rclcpp::Node("state_estimator_node") 
{
    RCLCPP_INFO(get_logger(), "Starting state_estimator_node follower node...");
}

void StateEstimatorNode::initialized(){
    init_subscription();
    init_service();
}

void StateEstimatorNode::init_subscription(){

    m_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        topic_sub::VEHICLE_GLOBAL_POSITION, 10, 
        std::bind(&StateEstimatorNode::current_gps_callback, this, _1)
    );

    m_current_setpoing_listener.subscribe(
        shared_from_this(), 
        topic_sub::VEHICLE_ODOMETRY, 10
    );
}

void StateEstimatorNode::init_service(){
    m_gps_to_local_srv = create_service<TransformGpsToLocal>(
        topic_srv::TRANSFORM_GPS_TO_LOCAL, 
        std::bind(&StateEstimatorNode::handle_gps_to_local, this, _1, _2));
}

void StateEstimatorNode::current_gps_callback(
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    double lat = msg->lat / 1e7;
    double lon = msg->lon / 1e7;
    double alt_m = msg->alt / 1e3; 
    bool lat_lon_valid = !std::isnan(msg->lat) && std::abs(msg->lat) > 0.1 &&
                     !std::isnan(msg->lon) && std::abs(msg->lon) > 0.1;

    bool alt_valid = !std::isnan(msg->alt) && std::abs(msg->alt) > 0.1;

    if (!m_geo_ref_status.reference_initialized && 
        lat_lon_valid &&
        alt_valid){
        m_geo_ref_status.reference_gps.lat = lat;
        m_geo_ref_status.reference_gps.lon = lon;
        m_geo_ref_status.reference_gps.alt = alt_m;
        m_geo_ref_status.reference_initialized = true;

        RCLCPP_INFO(this->get_logger(), "current_gps_callback: lat=%f, lon=%f, alt=%f",
                    lat, lon, alt_m);
    }
    m_geo_ref_status.current_gps.lat = lat;
    m_geo_ref_status.current_gps.lon = lon;
    m_geo_ref_status.current_gps.alt = alt_m;
}

void StateEstimatorNode::handle_gps_to_local(
        const std::shared_ptr<TransformGpsToLocal::Request> request,
        std::shared_ptr<TransformGpsToLocal::Response> response)
{
    if (!m_geo_ref_status.reference_initialized) {
        // RCLCPP_WARN(this->get_logger(), "Reference origin not initialized, cannot transform.");
        response->x = 0;
        response->y = 0;
        response->z = 0;
        return;
    }

    double x = 0.0, y = 0.0;
    geo_tool::gps_to_local(
        m_geo_ref_status.reference_gps.lat, 
        m_geo_ref_status.reference_gps.lon,
        request->latitude, request->longitude, 
        x, y
    );
    
    std::array<double, 3> target_position = {
        x, y, 
        m_geo_ref_status.reference_gps.lat - request->altitude
    };
    const float HORIZONTAL_DIST_THRESHOLD = 0.9f;
    const float VERTICAL_DIST_THRESHOLD = 0.4f;
    const auto &current_setpoint = m_current_setpoing_listener.get_msg();

    float dx = target_position[0] - current_setpoint.position[0];
    float dy = target_position[1] - current_setpoint.position[1];
    float dz = target_position[2] - current_setpoint.position[2];

    float horizontal_dist = std::hypot(dx, dy);
    float vertical_dist = std::abs(dz);
    float desired_yaw_rad = std::atan2(dy, dx);
    bool arrive = (horizontal_dist < HORIZONTAL_DIST_THRESHOLD) && (vertical_dist < VERTICAL_DIST_THRESHOLD);
    
    response->x = target_position[0];
    response->y = target_position[1];
    response->z = target_position[2];
    response->yaw = desired_yaw_rad;
    response->lat = m_geo_ref_status.current_gps.lat;
    response->lon = m_geo_ref_status.current_gps.lon;
    response->alt = m_geo_ref_status.current_gps.alt;
    response->arrive = arrive;
    
    // RCLCPP_INFO(get_logger(), "handle_gps_to_local vertical_dist: %f, horizontal_dist: %f", 
    // vertical_dist, horizontal_dist);

    // RCLCPP_INFO(this->get_logger(), "Converted GPS(%.6f, %.6f, %.2f) → ENU(%.2f, %.2f, %.2f)",
    //             request->latitude, request->longitude, request->altitude,
    //             x, y, response->z);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimatorNode>();
    node->initialized();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
