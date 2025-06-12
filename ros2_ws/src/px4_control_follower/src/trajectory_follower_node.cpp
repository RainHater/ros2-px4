#include "trajectory_follower_node.h"

TrajectoryFollower::TrajectoryFollower() : Node("trajectory_follower_node")
{
    m_offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    m_trajectory_setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    m_vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    m_goal_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&TrajectoryFollower::goal_pose_callback, this, std::placeholders::_1));

    m_offboard_setpoint_counter = 0;

    auto timer_callback = [this]() -> void {
        if (m_offboard_setpoint_counter == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

            // Arm the vehicle
            this->arm();
        }

        // offboard_control_mode needs to be paired with trajectory_setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();

        // stop the counter after reaching 11
        if (m_offboard_setpoint_counter < 11) {
            m_offboard_setpoint_counter++;
        }
    };

    m_timer = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

void TrajectoryFollower::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void TrajectoryFollower::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void TrajectoryFollower::publish_offboard_control_mode()
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

void TrajectoryFollower::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    // 如果目标位置可用，则更新目标位置为 trajectory setpoint
    if (m_goal_pose.pose.position.x != 0.0 || m_goal_pose.pose.position.y != 0.0 || m_goal_pose.pose.position.z != 0.0) {
        msg.position[0] = m_goal_pose.pose.position.x;
        msg.position[1] = m_goal_pose.pose.position.y;
        msg.position[2] = m_goal_pose.pose.position.z;
        msg.yaw = 0.0;  // 可以根据需要设置目标航向
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    }

    m_trajectory_setpoint_publisher->publish(msg);
}

void TrajectoryFollower::publish_vehicle_command(uint16_t command, float param1, float param2)
{
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

void TrajectoryFollower::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    m_goal_pose = *msg;
    RCLCPP_INFO(this->get_logger(), "take over pose information!");
    // RCLCPP_INFO(this->get_logger(), "Received goal pose: [x: %.2f, y: %.2f, z: %.2f]",
    //             m_goal_pose.position.x, m_goal_pose.position.y, m_goal_pose.position.z);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting trajectory follower node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryFollower>());

    rclcpp::shutdown();
    return 0;
}
