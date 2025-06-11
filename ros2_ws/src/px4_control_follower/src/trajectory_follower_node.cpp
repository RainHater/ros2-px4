#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class TrajectoryFollower : public rclcpp::Node
{
public:
    TrajectoryFollower() : Node("trajectory_follower_node")
    {
        // 发布器
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        // 订阅器：订阅目标轨迹
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&TrajectoryFollower::goal_pose_callback, this, std::placeholders::_1));

        // 定时器：每 100ms 发布控制模式和 setpoint
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryFollower::control_loop, this));

        // 启动 Offboard 模式
        send_vehicle_command(1, 1);   // ARM
        send_vehicle_command(176, 1); // PX4_CMD_DO_SET_MODE: PX4_CUSTOM_MAIN_MODE_OFFBOARD
    }

private:
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), "take over pose information!");
        last_goal_pose_ = *msg;
        has_new_goal_ = true;
    }

    void control_loop()
    {
        // 1. 发布控制模式
        px4_msgs::msg::OffboardControlMode mode{};
        mode.position = true;
        mode.velocity = false;
        mode.timestamp = now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(mode);

        // 2. 发布 setpoint
        if (has_new_goal_)
        {
            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = now().nanoseconds() / 1000;
            sp.position[0] = last_goal_pose_.pose.position.x;
            sp.position[1] = last_goal_pose_.pose.position.y;
            sp.position[2] = last_goal_pose_.pose.position.z;
            sp.yaw = 0.0;
            setpoint_pub_->publish(sp);
        }
    }

    void send_vehicle_command(uint16_t command, float param1)
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = now().nanoseconds() / 1000;
        cmd.param1 = param1;
        cmd.command = command;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        vehicle_command_pub_->publish(cmd);
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped last_goal_pose_;
    bool has_new_goal_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
