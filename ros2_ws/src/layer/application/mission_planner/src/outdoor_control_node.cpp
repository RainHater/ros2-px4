#include "mission_planner/outdoor_control_node.h"
#include <utilities/topic_name.hpp>

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;

OutdoorControlNode::OutdoorControlNode()
    : rclcpp::Node("outdoor_control_node")
{   
    m_fly = IDLE;
    RCLCPP_INFO(get_logger(), "outdoor_control_node 节点启动...");
}

void OutdoorControlNode::initialize(){

    init_pub();
    init_sub();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OutdoorControlNode::task_loop, this)
    );
}

void OutdoorControlNode::init_pub(){
    m_pub.offboard_mode = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_in::PX4_MODE, 10
    );

    m_pub.trajectory_setpoint = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_px4_in::PX4_TRAJECTORY_SETPOINT, 10
    );
}

void OutdoorControlNode::init_sub(){
    m_sub.offboard_mode.subscribe(
        shared_from_this(), 
        topic_out::PX4_MODE, 10
    );

    m_sub.vehicle_odometry.subscribe(
        shared_from_this(), 
        topic_px4_out::VEHICLE_ODOMETRY, 10
    );

    m_sub.vehicle_global_position.subscribe(
        shared_from_this(), 
        topic_px4_out::VEHICLE_GLOBAL_POSITION, 10
    );
}

void OutdoorControlNode::task_loop(){
    switch(m_fly){
        case IDLE:{
            m_interface.mode_control.unlock(
                ARM_ENABLE, POSITION, 
                m_sub.offboard_mode.get_msg(), 
                m_pub_msgs.offboard_mode
            );
            if (m_interface.mode_control.wait_busy()){
                m_fly = RISE;
                RCLCPP_INFO(get_logger(), "初始化完成!");
            }
            break;
        }
        case RISE:{
            if (!m_sub.vehicle_global_position.has_received())
                return;

            auto vehicle_global_position = m_sub.vehicle_global_position.get_first_msg();
            double lat = vehicle_global_position.lat;
            double lon = vehicle_global_position.lon;
            float  alt = vehicle_global_position.alt + 10;
            // RCLCPP_INFO(
            //     get_logger(), 
            //     "lat: %f, lon: %f, alt: %f",
            //     lat, lon, alt
            // );
            m_interface.gps_movement.move_to_gps_target(
                lat, lon, alt, 
                m_sub.vehicle_global_position.get_msg(),
                m_pub_msgs.offboard_mode
            );

            if (m_interface.gps_movement.switchflymode()){
                RCLCPP_INFO(get_logger(), "上升完成!");
            }
            // m_interface.movement.change_height(
            //     m_sub.vehicle_odometry.get_msg(),
            //     m_pub_msgs.trajectory_setpoint,
            //     get_clock()->now(),
            //     0.5,
            //     0.15
            // );
            // if (m_interface.movement.wait_busy()){
            //     m_fly = Hover;
            //     RCLCPP_INFO(get_logger(), "上升完成!");
            // }
            break;
        }

        case Hover:{
            // m_interface.movement.move_by_offset(
            //     m_sub.vehicle_odometry.get_msg(),
            //     m_pub_msgs.trajectory_setpoint,
            //     get_clock()->now(),
            //     {0, 0.5, 0},
            //     0.25, true
            // );
            // if (m_interface.movement.wait_busy()){
            //     m_fly = LAND;
            //     RCLCPP_INFO(get_logger(), "徘徊完成!");
            // }
            break;
        }
        case LAND:{
            // bool finish = m_interface.movement.land_mode(
            //     0.3,
            //     m_interface.mode_control,
            //     get_clock()->now(),
            //     m_sub.offboard_mode.get_msg(),
            //     m_sub.vehicle_odometry.get_msg(),
            //     m_pub_msgs.trajectory_setpoint,
            //     m_pub_msgs.offboard_mode,
            //     m_sub.local_position
            // );
            // if (finish){
            //     m_fly = END;
            //     RCLCPP_INFO(get_logger(), "降落完成!");
            // }
            break;
        }
        case END:{
            m_interface.mode_control.locked(
                m_sub.offboard_mode.get_msg(), 
                m_pub_msgs.offboard_mode
            );
            break;
        }
    }

    auto timestamp = get_clock()->now().nanoseconds() / 1000;

    m_pub_msgs.trajectory_setpoint.timestamp = timestamp;

    m_pub.offboard_mode->publish(m_pub_msgs.offboard_mode);
    if (m_sub.offboard_mode.get_msg().arm == ARM_ENABLE){
        m_pub.trajectory_setpoint->publish(m_pub_msgs.trajectory_setpoint);
    }
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OutdoorControlNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
