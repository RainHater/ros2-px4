#include "interface/px4_bridge_node.h"

using std::placeholders::_1;

PX4BridgeNode::PX4BridgeNode()
    : rclcpp::Node("px4_bridge_node") {
    RCLCPP_INFO(get_logger(), "Starting px4_bridge_node follower node...");
    
    init_px4_publisher();
    init_px4_subscription();
    init_external_publisher();
    init_external_subscription();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&PX4BridgeNode::timer_callback, this));
}

void PX4BridgeNode::init_px4_publisher(){
    m_vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    m_offboard_control_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
}

void PX4BridgeNode::init_px4_subscription(){
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_vehicle_odometry_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&PX4BridgeNode::vehicle_odometry_callback, this, _1)); 
    m_vehicle_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", qos,
        std::bind(&PX4BridgeNode::vehicle_global_position_callback, this, _1)); 
    m_vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos,
        std::bind(&PX4BridgeNode::vehicle_status_callback, this, _1)); 
}

void PX4BridgeNode::init_external_publisher(){
    m_vehicle_odometry_pub = create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/interface/out/vehicle_odometry", 10);
    m_vehicle_global_position_pub = create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
        "/interface/out/vehicle_global_position", 10);
    m_vehicle_status_pub = create_publisher<px4_msgs::msg::VehicleStatus>(
        "/interface/out/vehicle_status", 10);
}

void PX4BridgeNode::init_external_subscription(){
    m_vehicle_command_sub = create_subscription<px4_msgs::msg::VehicleCommand>(
        "/interface/in/vehicle_command", 10,
        std::bind(&PX4BridgeNode::vehicle_command_callback, this, _1)); 
    m_trajectory_setpoint_sub = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10,
        std::bind(&PX4BridgeNode::trajectory_setpoint_callback, this, _1)); 
    m_offboard_control_mode_sub = create_subscription<px4_msgs::msg::OffboardControlMode>(
        "/interface/in/offboard_control_mode", 10,
        std::bind(&PX4BridgeNode::offboard_control_mode_callback, this, _1)); 
}

void PX4BridgeNode::timer_callback(){
    
}

void PX4BridgeNode::vehicle_command_callback(const px4_msgs::msg::VehicleCommand &msg){
    m_vehicle_command_pub->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Forwarded VehicleCommand: command=%u", msg.command);
}

void PX4BridgeNode::trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint &msg){
    m_trajectory_setpoint_pub->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Published TrajectorySetpoint: x=%.2f y=%.2f z=%.2f",
                    msg.position[0],
                    msg.position[1],
                    msg.position[2]);
}

void PX4BridgeNode::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry &msg){
    m_vehicle_odometry_pub->publish(msg);
}

void PX4BridgeNode::offboard_control_mode_callback(const px4_msgs::msg::OffboardControlMode &msg){
    m_offboard_control_mode_pub->publish(msg);
}

void PX4BridgeNode::vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition &msg){
    m_vehicle_global_position_pub->publish(msg);
}

void PX4BridgeNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus &msg){
    m_vehicle_status_pub->publish(msg);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4BridgeNode>());

    rclcpp::shutdown();
    return 0;
}
