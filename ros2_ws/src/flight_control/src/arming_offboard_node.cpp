#include "arm_offboard/arming_offboard_node.h"

ArmingOffboardNode::ArmingOffboardNode() : Node("arming_offboard_node"){

    m_offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    m_vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    m_offboard_setpoint_counter = 0;

    auto timer_callback = [this]() -> void {
        if (m_offboard_setpoint_counter == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this->arm();
        }

        publish_offboard_control_mode();

        // stop the counter after reaching 11
        if (m_offboard_setpoint_counter < 11) {
            m_offboard_setpoint_counter++;
        }
    };

    m_timer = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

void ArmingOffboardNode::publish_vehicle_command(uint16_t command, float param1, float param2){

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
    m_vehicle_command_publisher->publish(msg);
}

void ArmingOffboardNode::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    m_offboard_control_mode_publisher->publish(msg);
}

void ArmingOffboardNode::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void ArmingOffboardNode::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

int main(int argc, char *argv[])
{
    std::cout << "Starting ArmingOffboardNode follower node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmingOffboardNode>());

    rclcpp::shutdown();
    return 0;
}
