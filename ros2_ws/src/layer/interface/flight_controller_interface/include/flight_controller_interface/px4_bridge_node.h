#ifndef _PX4_BRIDGE_NODE_H
#define _PX4_BRIDGE_NODE_H

#include <px4_msgs/msg/detail/battery_status__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "flight_controller_interface/ros_topic_forwarder.hpp"
#include "flight_controller_interface/px4_topic_name.hpp"

#include "utilities/topic_name.hpp"

class PX4BridgeNode : public rclcpp::Node {
public:
    PX4BridgeNode();
    void initialized();
protected:
    void init_forwarder();
    void init_forwarder_px4();
private:
    //飞控转发外部
    TopicForwarder<px4_msgs::msg::VehicleOdometry> m_vehicle_odometry_pair;
    TopicForwarder<px4_msgs::msg::VehicleGlobalPosition> m_vehicle_global_position_pair;
    TopicForwarder<px4_msgs::msg::VehicleStatus> m_vehicle_status_pair;
    TopicForwarder<px4_msgs::msg::VehicleLocalPosition> m_vehicle_local_position_pair;
    TopicForwarder<px4_msgs::msg::BatteryStatus> m_battery_status_pair;

    //外部转发飞控
    TopicForwarder<px4_msgs::msg::VehicleCommand> m_vehicle_command_px4;
    TopicForwarder<px4_msgs::msg::TrajectorySetpoint> m_trajectory_setpoint_px4;
    TopicForwarder<px4_msgs::msg::OffboardControlMode> m_offboard_control_mode_px4;
};

#endif
