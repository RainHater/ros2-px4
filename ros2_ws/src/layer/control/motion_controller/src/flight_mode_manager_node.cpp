#include "motion_controller/flight_mode_manager_node.h"
#include "utilities/topic_pub_tool.hpp"
#include "utilities/topic_sub_tool.hpp"

constexpr auto ARMING_STATE_ARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
constexpr auto ARMING_STATE_DISARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_DISARMED;
constexpr auto OFFBOARD_NOT_ACTIVE = common_msgs::msg::ArmOffboardStatus::OFFBOARD_NOT_ACTIVE;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
constexpr auto ATTITUDE = common_msgs::msg::ArmOffboardStatus::ATTITUDE;
constexpr auto PX4_OFFBOARD_DEFAULT_MODE = POSITION;
constexpr auto PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
constexpr auto LOCK_INTERVAL_TIME = 10;

using std::placeholders::_1;

FlightModeManagerNode::FlightModeManagerNode() 
    : Node("flight_mode_manager_node"){
    RCLCPP_INFO(this->get_logger(), "Starting flight_mode_manager_node follower node...");
    
    m_offboard_setpoint_counter = 0;
    m_px4_mode.lock_interval_cnt = 0;
    m_px4_mode.current.arming_state = ARMING_STATE_DISARMED;
    m_px4_mode.current.offboard_mode = OFFBOARD_NOT_ACTIVE;
    m_px4_mode.target.arming_state = ARMING_STATE_ARMED;
    m_px4_mode.target.offboard_mode = PX4_OFFBOARD_DEFAULT_MODE;
}

void FlightModeManagerNode::initialize(){
    init_publisher();
    init_subscription();

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&FlightModeManagerNode::timer_callback, this));
}

void FlightModeManagerNode::init_publisher(){
    topic_pub_tool::offboard_control_mode(
        shared_from_this(), m_offboard_control_mode_pub);
    topic_pub_tool::vehicle_command(
        shared_from_this(), m_vehicle_command_pub);
    topic_pub_tool::control_px4_mode_status(
        shared_from_this(), m_px4_mode_status_broadcaster_pub);
}

void FlightModeManagerNode::init_subscription(){
    topic_sub_tool::control_set_offboard_mode(
        shared_from_this(), m_set_px4_mode_status_sub, 
        std::bind(&FlightModeManagerNode::set_px4_mode_status_callback, this, _1)
    );

    topic_sub_tool::vehicle_status(
        shared_from_this(), m_px4_mode_status_broadcaster_sub, 
        std::bind(&FlightModeManagerNode::px4_mode_status_broadcaster_callback, this, _1)
    );
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

void FlightModeManagerNode::set_px4_mode_status_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg){
    m_px4_mode.target.offboard_mode = msg->offboard_mode;
    RCLCPP_INFO(get_logger(), "current offboard mode: %d", m_px4_mode.current.offboard_mode);
}

void FlightModeManagerNode::px4_mode_status_broadcaster_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg){
    if (msg->arming_state == 1){
        m_px4_mode.current.arming_state = ARMING_STATE_DISARMED;
        if (m_offboard_setpoint_counter == 11 || msg->nav_state != 14){
            m_px4_mode.lock_interval_cnt ++;
            if (m_px4_mode.lock_interval_cnt >= LOCK_INTERVAL_TIME){
                m_offboard_setpoint_counter = 0;
                m_px4_mode.lock_interval_cnt = 0;
            }
        }
    }else if (msg->arming_state == 2){
        m_px4_mode.current.arming_state = ARMING_STATE_ARMED;
    }
    if (msg->nav_state == 14){
        m_px4_mode.current.offboard_mode = m_px4_mode.target.offboard_mode;
    }
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
    auto &mode = m_px4_mode.target.offboard_mode;
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = (mode == POSITION);
    msg.velocity = (mode == VELOCITY);
    msg.attitude = (mode == ATTITUDE);
    msg.acceleration = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    m_offboard_control_mode_pub->publish(msg);
}

void FlightModeManagerNode::publish_current_offboard_mode(){
    m_px4_mode_status_broadcaster_pub->publish(m_px4_mode.current);
}

void FlightModeManagerNode::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void FlightModeManagerNode::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightModeManagerNode>();
    node->initialize();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
