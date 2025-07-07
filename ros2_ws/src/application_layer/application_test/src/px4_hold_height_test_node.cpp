#include "application_test/px4_hold_height_test_node.h"

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

    init_publisher();
    init_subscription();
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&Px4HoldHeightTestNode::timer_callback, this));
}

void Px4HoldHeightTestNode::init_publisher(){
    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
    m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        "/control/set_offboard_mode", 10);
}

void Px4HoldHeightTestNode::init_subscription(){
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", 10, 
        std::bind(&Px4HoldHeightTestNode::px4_mode_status_callback, this, _1));
}

void Px4HoldHeightTestNode::timer_callback(){
    auto &arm_state = m_px4_current_mode.arming_state;
    auto &offboard_mode = m_px4_current_mode.offboard_mode;

    if (arm_state == ARMING_STATE_ARMED && 
        offboard_mode == POSITION)
    {
        common_msgs::msg::TrajectorySetPoint msg{};
        msg.position[0] = 0;
        msg.position[1] = 0;
        msg.position[2] = -1;
        m_trajectory_set_point_pub->publish(msg);
    }else {
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

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4HoldHeightTestNode>());
    rclcpp::shutdown();
    return 0;
}
