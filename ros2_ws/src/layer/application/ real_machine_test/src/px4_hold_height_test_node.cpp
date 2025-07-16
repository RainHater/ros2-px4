#include "application_test/px4_hold_height_test_node.h"
#include "utilities/topic_tool.hpp"
#include "utilities/tf2_tool.hpp"
#include "utilities/topic_pub_tool.hpp"
#include "utilities/topic_sub_tool.hpp"

constexpr auto ARMING_STATE_ARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
constexpr auto ARMING_STATE_DISARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_DISARMED;
constexpr auto OFFBOARD_NOT_ACTIVE = common_msgs::msg::ArmOffboardStatus::OFFBOARD_NOT_ACTIVE;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
constexpr auto ATTITUDE = common_msgs::msg::ArmOffboardStatus::ATTITUDE;

using std::placeholders::_1;

Px4HoldHeightTestNode::Px4HoldHeightTestNode()
    : rclcpp::Node("px4_hold_height_test_node") 
{
    RCLCPP_INFO(get_logger(), "Starting px4_hold_height_test_node follower node...");
}

void Px4HoldHeightTestNode::initialize(){
    init_publisher();
    init_subscription();
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&Px4HoldHeightTestNode::timer_callback, this));
}

void Px4HoldHeightTestNode::init_publisher(){
    topic_pub_tool::control_trajectory_setpoint(
        shared_from_this(), m_trajectory_set_point_pub);
    topic_pub_tool::control_set_offboard_mode(
        shared_from_this(), m_set_offboard_mode_pub);
}

void Px4HoldHeightTestNode::init_subscription(){
    topic_sub_tool::control_px4_mode_status(
        shared_from_this(), m_px4_mode_status_sub,
        std::bind(&Px4HoldHeightTestNode::px4_mode_status_callback, this, _1));

    topic_sub_tool::vehicle_local_position(
        shared_from_this(), m_vehicle_local_position_sub,
        std::bind(&Px4HoldHeightTestNode::vehicle_local_position_callback, this, _1));

    topic_sub_tool::vehicle_odometry(
        shared_from_this(), m_current_setpoing_sub,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
            m_current_setpoint = *msg;
        });
}

void Px4HoldHeightTestNode::timer_callback(){
    auto &arm_state = m_px4_current_mode.arming_state;
    auto &offboard_mode = m_px4_current_mode.offboard_mode;

    if (arm_state == ARMING_STATE_ARMED 
        && offboard_mode == POSITION)
    {   
        tf2_tool::EulerAngles angle{};
        tf2_tool::get_euler_angles(m_current_setpoint, angle);
        // RCLCPP_INFO(get_logger(), "yaw: %f", angle.yaw);

        common_msgs::msg::TrajectorySetPoint msg{};
        msg.position[0] = 0.0;
        msg.position[1] = 0.0;
        msg.position[2] = -0.5;
        msg.yaw = angle.yaw;
        m_trajectory_set_point_pub->publish(msg);
    } else {
        common_msgs::msg::ArmOffboardStatus msg{};
        msg.offboard_mode = POSITION;
        m_set_offboard_mode_pub->publish(msg);
    }
}

void Px4HoldHeightTestNode::px4_mode_status_callback(
    const common_msgs::msg::ArmOffboardStatus::SharedPtr msg)
{
    m_px4_current_mode = *msg;
}

void Px4HoldHeightTestNode::vehicle_local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){
    m_velocity_y = msg->vy;
    m_velocity_x = msg->vy;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);\
    auto node = std::make_shared<Px4HoldHeightTestNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
