#include "flight_controller_interface/px4_bridge_node.h"

PX4BridgeNode::PX4BridgeNode()
    : rclcpp::Node("px4_bridge_node") 
{   
    RCLCPP_INFO(get_logger(), "px4_bridge_node 节点启动...");
}

void PX4BridgeNode::initialized(){
    init_forwarder();
    init_forwarder_px4();
}

void PX4BridgeNode::init_forwarder(){
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_vehicle_odometry_pair.create_forwarding_subscription(
        shared_from_this(), 
        topic_px4_out::VEHICLE_ODOMETRY,
        topic_out::VEHICLE_ODOMETRY,
        qos
    );

    m_vehicle_status_pair.create_forwarding_subscription(
        shared_from_this(), 
        topic_px4_out::VEHICLE_STATUS,
        topic_out::VEHICLE_STATUS,
        qos
    );

    m_vehicle_global_position_pair.create_forwarding_subscription(
        shared_from_this(), 
        topic_px4_out::VEHICLE_GLOBAL_POSITION,
        topic_out::VEHICLE_GLOBAL_POSITION,
        qos
    );

    m_vehicle_local_position_pair.create_forwarding_subscription(
        shared_from_this(), 
        topic_px4_out::VEHICLE_LOCAL_POSITION,
        topic_out::VEHICLE_LOCAL_POSITION,
        qos
    );

    m_battery_status_pair.create_forwarding_subscription(
        shared_from_this(), 
        topic_px4_out::BATTERY_STATUS,
        topic_out::BATTERY_STATUS,
        qos
    );
}

void PX4BridgeNode::init_forwarder_px4(){
    m_vehicle_command_px4.create_forwarding_subscription(
        shared_from_this(), 
        topic_in::VEHICLE_COMMAND, 
        topic_px4_in::VEHICLE_COMMAND,
        10
    );

    m_trajectory_setpoint_px4.create_forwarding_subscription(
        shared_from_this(), 
        topic_in::PX4_TRAJECTORY_SETPOINT, 
        topic_px4_in::TRAJECTORY_SETPOINT,
        10
    );

    m_offboard_control_mode_px4.create_forwarding_subscription(
        shared_from_this(), 
        topic_in::OFFBOARD_CONTROL_MODE, 
        topic_px4_in::OFFBOARD_CONTROL_MODE,
        10
    );
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4BridgeNode>();
    node->initialized();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
