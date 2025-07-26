#include "mission_planner/vision_test_node.h"

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

VisionTestNode::VisionTestNode()
    : rclcpp::Node("vision_test_node")
{
    RCLCPP_INFO(get_logger(), "vision_test_node 节点启动...");
}

void VisionTestNode::initialize(){

    init_pub();
    init_sub();

    m_timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisionTestNode::task_loop, this));
}

void VisionTestNode::init_pub(){
    m_pub.offboard_mode = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_pub::PX4_MODE, 10
    );

    m_pub.trajectory_setpoint = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_pub::PX4_TRAJECTORY_SETPOINT, 10
    );
}

void VisionTestNode::init_sub(){
    m_sub.offboard_mode.subscribe(
        shared_from_this(), 
        topic_sub::PX4_MODE, 10
    );

    m_sub.vehicle_odometry.subscribe(
        shared_from_this(), 
        topic_sub::VEHICLE_ODOMETRY, 10
    );
}

void VisionTestNode::task_loop(){
    switch(m_fly){
        case IDLE:{
            m_interface.mode_control.unlock(
                ARM_ENABLE, POSITION, 
                m_sub.offboard_mode.get_msg(), 
                m_pub_msgs.offboard_mode
            );
            if (m_interface.mode_control.wait_busy()){
                m_fly = TO2Hover;
                RCLCPP_INFO(get_logger(), "初始化完成!");
            }
            break;
        }
        case TO2Hover:{
            Waypts start = {0, 0, 0};
            Waypts end = {0, 0, -3};

            m_interface.movement.justmove(
                m_sub.vehicle_odometry.get_msg(),
                m_pub_msgs.trajectory_setpoint,
                get_clock()->now(),
                start, end,
                0.25, 25
            );
            if (m_interface.movement.wait_busy()){
                m_fly = Hover;
                RCLCPP_INFO(get_logger(), "上升完成!");
            }
            break;
        }

        case Hover:{
            Waypts start = {0, 0, 0};
            Waypts end = {4, 1, -3};

            m_interface.movement.justmove(
                m_sub.vehicle_odometry.get_msg(),
                m_pub_msgs.trajectory_setpoint,
                get_clock()->now(),
                start, end,
                0.25, 25
            );
            if (m_interface.movement.wait_busy()){
                m_fly = Hover;
                RCLCPP_INFO(get_logger(), "飞行完成!");
            }
            break;
        }
    }

    auto timestamp = get_clock()->now().nanoseconds() / 1000;

    m_pub_msgs.trajectory_setpoint.timestamp = timestamp;

    m_pub.offboard_mode->publish(m_pub_msgs.offboard_mode);
    m_pub.trajectory_setpoint->publish(m_pub_msgs.trajectory_setpoint);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionTestNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
