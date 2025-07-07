#include "motion_controller/flight_mode_manager_node.h"

using std::placeholders::_1;

FlightModeManagerNode::FlightModeManagerNode() 
    : Node("flight_mode_manager_node"){
    RCLCPP_INFO(this->get_logger(), "Starting flight_mode_manager_node follower node...");
    
    m_offboard_setpoint_counter = 0;
    m_current_mode.arming_state = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_DISARMED;
    m_current_mode.offboard_mode = PX4_OFFBOARD_DEFAULT_MODE;
        
    init_publisher();
    init_subscription();

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&FlightModeManagerNode::timer_callback, this));
}

void FlightModeManagerNode::init_publisher(){
    m_offboard_control_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/interface/in/offboard_control_mode", 10);
    m_vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/interface/in/vehicle_command", 10);
    m_px4_mode_status_broadcaster_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10);
}

void FlightModeManagerNode::init_subscription(){
    m_set_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/set_offboard_mode", 10, 
        std::bind(&FlightModeManagerNode::set_px4_mode_status_callback, this, _1));
    
    m_px4_mode_status_broadcaster_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/interface/out/vehicle_status_v1", 10, 
        std::bind(&FlightModeManagerNode::px4_mode_status_broadcaster_callback, this, _1));
}

void FlightModeManagerNode::timer_callback(){
    if (m_offboard_setpoint_counter == 10) {
        // Change to Offboard mode after 10 setpoints
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        // Arm the vehicle
        arm();
    }

    publish_px4_offboard_mode();
    publish_current_offboard_mode();

    // stop the counter after reaching 11
    if (m_offboard_setpoint_counter < 11){
        m_offboard_setpoint_counter++;
    }
}

void FlightModeManagerNode::set_px4_mode_status_callback(
    const common_msgs::msg::ArmOffboardStatus &msg)
{
    m_current_mode.offboard_mode = msg.offboard_mode;
    RCLCPP_INFO(get_logger(), "current offboard mode: %d", m_current_mode.offboard_mode);
}

void FlightModeManagerNode::px4_mode_status_broadcaster_callback(const px4_msgs::msg::VehicleStatus &msg){
    (void)msg;
}

void FlightModeManagerNode::publish_vehicle_command(uint16_t command, float param1, float param2){
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    m_vehicle_command_pub->publish(msg);
}

void FlightModeManagerNode::publish_px4_offboard_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = (m_current_mode.offboard_mode == common_msgs::msg::ArmOffboardStatus::POSITION);
    msg.velocity = (m_current_mode.offboard_mode == common_msgs::msg::ArmOffboardStatus::VELOCITY);
    msg.attitude = (m_current_mode.offboard_mode == common_msgs::msg::ArmOffboardStatus::ATTITUDE);
    msg.acceleration = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    m_offboard_control_mode_pub->publish(msg);
}

void FlightModeManagerNode::publish_current_offboard_mode(){
    m_px4_mode_status_broadcaster_pub->publish(m_current_mode);
}

void FlightModeManagerNode::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    m_current_mode.arming_state = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void FlightModeManagerNode::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightModeManagerNode>());

    rclcpp::shutdown();
    return 0;
}
