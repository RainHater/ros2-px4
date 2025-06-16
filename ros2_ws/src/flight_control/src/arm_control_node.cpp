// src/flight_control/src/arm_control_node.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "utilities/vehicle_command_helper.h"

using std::placeholders::_1;
using std::placeholders::_2;

class ArmControlNode : public rclcpp::Node
{
public:
  ArmControlNode()
  : Node("arm_control_node"), m_cmd_helper(this)
  {
    // Publisher: VehicleCommand
    m_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::SystemDefaultsQoS());

    // Service: /arm_trigger （std_srvs/Trigger）
    m_srv = this->create_service<std_srvs::srv::Trigger>(
      "arm_trigger",
      std::bind(&ArmControlNode::on_arm_request, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "ArmControlNode ready. Call 'ros2 service call /arm_trigger std_srvs/srv/Trigger'");
  }

private:
  void on_arm_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    auto cmd = px4_msgs::msg::VehicleCommand();

    // param1 = 1.0 -> arm; 0.0 -> disarm
    cmd.param1 = 1.0;
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = 1;       // PX4 系统 ID
    cmd.target_component = 1;    // 通常组件 ID=1
    cmd.source_system = 1;       // 本机系统 ID
    cmd.source_component = 1;    // 本机组件 ID
    cmd.from_external = true;    // 外部命令

    m_cmd_pub->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Sent ARM command");

    res->success = true;
    res->message = "Arm command sent";
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_cmd_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srv;
  utilities::VehicleCommandHelper m_cmd_helper;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControlNode>());
  rclcpp::shutdown();
  return 0;
}
