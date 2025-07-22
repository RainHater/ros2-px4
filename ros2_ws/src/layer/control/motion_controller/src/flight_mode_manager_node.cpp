#include "motion_controller/flight_mode_manager_node.h"
#include "utilities/topic_name.hpp"

using std::placeholders::_1;

FlightModeManagerNode::FlightModeManagerNode() 
    : Node("flight_mode_manager_node")
{   
    RCLCPP_INFO(get_logger(), "flight_mode_manager_node 节点启动...");
    
    m_offboard_setpoint_counter = 0;
    m_px4_mode.lock_interval_time = 0;
    m_px4_mode.current.arming_state = ARMING_STATE_DISARMED;
    m_px4_mode.current.offboard_mode = OFFBOARD_NOT_ACTIVE;
    m_px4_mode.target.arming_state = ARMING_STATE_ARMED;
    m_px4_mode.target.offboard_mode = PX4_OFFBOARD_DEFAULT_MODE;
    m_px4_mode.last.offboard_mode = m_px4_mode.current.offboard_mode;
}

void FlightModeManagerNode::initialize(){
    init_publisher();
    init_subscription();

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&FlightModeManagerNode::timer_callback, this));
}

void FlightModeManagerNode::init_publisher(){
    m_offboard_control_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
        topic_pub::OFFBOARD_CONTROL_MODE, 10);

    m_vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
        topic_pub::VEHICLE_COMMAND, 10);

    m_px4_mode_status_broadcaster_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_pub::PX4_MODE_STATUS, 10);
}

void FlightModeManagerNode::init_subscription(){
    m_set_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        topic_sub::SET_OFFBOARD_MODE, 10, 
        std::bind(&FlightModeManagerNode::set_px4_mode_status_callback, this, _1)
    );

    m_px4_mode_status_broadcaster_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
        topic_sub::VEHICLE_STATUS, 10, 
        std::bind(&FlightModeManagerNode::px4_mode_status_broadcaster_callback, this, _1)
    );
}

void FlightModeManagerNode::timer_callback(){
    m_px4_mode.state_release(shared_from_this());

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
}

void FlightModeManagerNode::px4_mode_status_broadcaster_callback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    auto arm_mode = msg->arming_state;
    auto offboard = msg->nav_state;
    auto now_time = get_clock()->now().nanoseconds() / 1000000;
    auto interval = now_time - m_px4_mode.lock_interval_time;
    
    if ((arm_mode == PX4_ARMING_STATE_DISARMED
        || offboard != NAVIGATION_STATE_OFFBOARD)
        && m_offboard_setpoint_counter == 11
        && interval >= LOCK_INTERVAL_TIMER)
    {
        m_offboard_setpoint_counter = 0;
    }

    m_px4_mode.current.arming_state = (arm_mode==PX4_ARMING_STATE_DISARMED)
        ?ARMING_STATE_DISARMED:ARMING_STATE_ARMED;
    m_px4_mode.current.offboard_mode = (offboard==NAVIGATION_STATE_OFFBOARD)
        ?m_px4_mode.target.offboard_mode:m_px4_mode.current.offboard_mode;

    if (interval >= LOCK_INTERVAL_TIMER){
        m_px4_mode.lock_interval_time = now_time;
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
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_offboard_control_mode_pub->publish(msg);
}

void FlightModeManagerNode::publish_current_offboard_mode(){
    m_px4_mode_status_broadcaster_pub->publish(m_px4_mode.current);
}

void FlightModeManagerNode::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_DEBUG(get_logger(), "Arm command send");
}

void FlightModeManagerNode::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_DEBUG(get_logger(), "Disarm command send");
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
