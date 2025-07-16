#ifndef _PX4_BRIDGE_NODE_H
#define _PX4_BRIDGE_NODE_H

#include <px4_msgs/msg/detail/vehicle_status__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PX4BridgeNode : public rclcpp::Node {
public:
    PX4BridgeNode();
protected:
    //飞控发布
    void init_px4_publisher();
    //飞控订阅
    void init_px4_subscription();
    //外部发布
    void init_external_publisher();
    //外部订阅
    void init_external_subscription();
    void timer_callback();
    void vehicle_command_callback(const px4_msgs::msg::VehicleCommand &msg);
    void trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint &msg);
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry &msg);
    void offboard_control_mode_callback(const px4_msgs::msg::OffboardControlMode &msg);
    void vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition &msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus &msg);
    void vehicle_local_position_callbacks(const px4_msgs::msg::VehicleLocalPosition &msg);
private:

    rclcpp::TimerBase::SharedPtr m_timer;
    
    //start 对于飞控发布
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_vehicle_command_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboard_control_mode_pub;
    //end 对于飞控发布

    //start 对于飞控订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_vehicle_odometry_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_vehicle_global_position_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr m_vehicle_status_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr m_vehicle_local_position_sub;
    //end 对于飞控订阅

    //start 对于外部发布
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr m_vehicle_odometry_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_vehicle_global_position_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr m_vehicle_status_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr m_vehicle_local_position_pub;
    //end 对于外部发布

    //start 对于外部订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr m_vehicle_command_sub;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectory_setpoint_sub;
    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboard_control_mode_sub;
    //end 对于外部订阅
};

#endif
