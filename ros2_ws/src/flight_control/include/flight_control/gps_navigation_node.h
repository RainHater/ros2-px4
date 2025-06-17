#ifndef _GPS_NAVIGATION_NODE_H
#define _GPS_NAVIGATION_NODE_H

#include <common_msgs/msg/detail/position_setpoint__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/target_gps.hpp>
#include <common_msgs/msg/position_setpoint.hpp>

class GpsNavigationNode : public rclcpp::Node {
public:
    GpsNavigationNode();
protected:
    //自身经纬度回调函数
    void target_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    //目标经纬度回调函数
    void target_gps_callback(const common_msgs::msg::TargetGps::SharedPtr msg);
    //经纬度转换
    void convert_gps_to_position(const common_msgs::msg::TargetGps::SharedPtr msg);
    //发布位置
    void publish_target_position(double dlat, double dlon, double dz);
private:
    //自身经纬度订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_global_position_sub;
    //目标经纬度订阅
    rclcpp::Subscription<common_msgs::msg::TargetGps>::SharedPtr m_target_gps;
    //发布位置
    rclcpp::Publisher<common_msgs::msg::PositionSetpoint>::SharedPtr m_target_position_pub;
    //当前经纬度消息
    common_msgs::msg::TargetGps m_current_gps;
    //常量
    const double m_deg_to_m = 111320.0;
};

#endif
