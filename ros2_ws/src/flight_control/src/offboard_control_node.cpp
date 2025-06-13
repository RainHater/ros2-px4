#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class OffboardControlNode : public rclcpp::Node
{
public:
    OffboardControlNode()
    : Node("offboard_control_node")
    {
        // 创建发布器
        m_offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", rclcpp::QoS(10));
        
        m_trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", rclcpp::QoS(10));
        m_goal_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/goal_pose", 10, std::bind(&OffboardControlNode::goal_pose_callback, this, std::placeholders::_1));
        // 定时器定时发布
        m_timer = this->create_wall_timer(
        100ms, std::bind(&OffboardControlNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "OffboardControlNode started");
    }

private:
    void timer_callback() {
        // 发布控制模式：仅启用位置控制
        px4_msgs::msg::OffboardControlMode ctrl_msg;
        ctrl_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        ctrl_msg.position = true;
        ctrl_msg.velocity = false;
        ctrl_msg.acceleration = false;
        ctrl_msg.attitude = false;
        ctrl_msg.body_rate = false;
        m_offboard_control_mode_pub->publish(ctrl_msg);

        if (m_goal_pose.pose.position.x != 0.0 || m_goal_pose.pose.position.y != 0.0 || m_goal_pose.pose.position.z != 0.0) {
            px4_msgs::msg::TrajectorySetpoint sp_msg;
            sp_msg.position[0] = m_goal_pose.pose.position.x;
            sp_msg.position[1] = m_goal_pose.pose.position.y;
            sp_msg.position[2] = m_goal_pose.pose.position.z;
            sp_msg.yaw = 0.0;  // 可以根据需要设置目标航向
            sp_msg.timestamp = ctrl_msg.timestamp;
        }
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        m_goal_pose = *msg;
        RCLCPP_INFO(this->get_logger(), "take over pose information!");
        // RCLCPP_INFO(this->get_logger(), "Received goal pose: [x: %.2f, y: %.2f, z: %.2f]",
        //             m_goal_pose.position.x, m_goal_pose.position.y, m_goal_pose.position.z);
    }


    //订阅器：接收目标位置的消息
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_subscription;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboard_control_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    //目标位置
    geometry_msgs::msg::PoseStamped m_goal_pose;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlNode>());
  rclcpp::shutdown();
  return 0;
}
