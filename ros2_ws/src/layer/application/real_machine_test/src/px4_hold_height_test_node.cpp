#include "application_test/px4_hold_height_test_node.h"
#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"
#include <chrono>

constexpr auto ARMING_STATE_ARMED = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto LAND = common_msgs::msg::ArmOffboardStatus::LAND;

using std::placeholders::_1;

Px4HoldHeightTestNode::Px4HoldHeightTestNode()
    : rclcpp::Node("px4_hold_height_test_node") 
{
    RCLCPP_INFO(get_logger(), "Starting px4_hold_height_test_node follower node...");
    m_task_state = TASK1;
}

void Px4HoldHeightTestNode::initialize(){
    init_publisher();
    init_subscription();

    m_1s_timer = create_wall_timer(std::chrono::seconds(1),
    [this](){
        m_gobal_1s_timer ++;
    });

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&Px4HoldHeightTestNode::timer_callback, this));
    RCLCPP_INFO(get_logger(), "task start");
}

void Px4HoldHeightTestNode::init_publisher(){
    m_trajectory_set_point_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_pub::TRAJECTORY_SETPOINT, 10);

    m_set_offboard_mode_pub = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_pub::SET_OFFBOARD_MODE, 10);
}

void Px4HoldHeightTestNode::init_subscription(){
    m_px4_mode_status_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        topic_sub::PX4_MODE_STATUS, 10, 
        std::bind(&Px4HoldHeightTestNode::px4_mode_status_callback, this, _1)
    );

    m_current_setpoing_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        topic_sub::VEHICLE_ODOMETRY, 10, 
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
            m_current_setpoint = *msg;
        }
    );
}

void Px4HoldHeightTestNode::timer_callback(){
    auto &arm_state = m_px4_current_mode.arming_state;
    auto &offboard_mode = m_px4_current_mode.offboard_mode;

    if (m_task_state == TASK1){
        common_msgs::msg::ArmOffboardStatus msg{};
        msg.offboard_mode = POSITION;
        m_set_offboard_mode_pub->publish(msg);
        if (arm_state == ARMING_STATE_ARMED 
            && offboard_mode == POSITION)
        {   
            m_gobal_1s_timer = 0;
            m_task_state = TASK2;
            RCLCPP_INFO(get_logger(), "switch task 2");
        }
    }else if (m_task_state == TASK2){
        tf2_tool::EulerAngles angle{};
        tf2_tool::get_euler_angles(m_current_setpoint, angle);
        common_msgs::msg::TrajectorySetPoint msg{};
        msg.position[0] = 0.0;
        msg.position[1] = 0.0;
        msg.position[2] = -0.5;
        msg.yaw = angle.yaw;
        m_trajectory_set_point_pub->publish(msg);
        if (m_gobal_1s_timer >= 10){
            m_task_state = TASK4;
            m_gobal_1s_timer = 0;
            RCLCPP_INFO(get_logger(), "switch task 3");
        }
    }else if (m_task_state == TASK3){
        tf2_tool::EulerAngles angle{};
        tf2_tool::get_euler_angles(m_current_setpoint, angle);
        common_msgs::msg::TrajectorySetPoint msg{};
        msg.position[0] = 0.0;
        msg.position[1] = 0.5;
        msg.position[2] = -0.5;
        msg.yaw = angle.yaw;
        m_trajectory_set_point_pub->publish(msg);
        if (m_gobal_1s_timer >= 5){
            m_task_state = TASK4;
            m_gobal_1s_timer = 0;
            RCLCPP_INFO(get_logger(), "switch task 4");
        }
    }else if (m_task_state == TASK4){
        common_msgs::msg::ArmOffboardStatus msg{};
        msg.offboard_mode = LAND;
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
    auto node = std::make_shared<Px4HoldHeightTestNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
