#ifndef _GPS_NAVIGATION_NODE_H
#define _GPS_NAVIGATION_NODE_H

#include <common_msgs/msg/detail/position_setpoint__struct.hpp>
#include <common_msgs/msg/detail/trajectory_set_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/target_gps.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>

class GpsNavigationNode : public rclcpp::Node {
public:
    GpsNavigationNode();
protected:
    //定时器回调函数
    void timer_callback();
    //自身经纬度回调函数
    void current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    //目标经纬度回调函数
    void target_gps_callback(const common_msgs::msg::TargetGps::SharedPtr msg);
    //经纬度转换
    void convert_gps_to_position();
    //发布位置
    void publish_target_position(double dlat, double dlon, double dz);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    //自身经纬度订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_global_position_sub;
    //目标经纬度订阅
    rclcpp::Subscription<common_msgs::msg::TargetGps>::SharedPtr m_target_gps_sub;
    //发布位置
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_target_position_pub;
    //目标经纬度消息
    common_msgs::msg::TargetGps m_target_gps;
    //当前经纬度消息
    common_msgs::msg::TargetGps m_current_gps;
    //常量
    const double m_R = 6'371'000.0;
    //起点海拔
    double m_home_alt;
    //转换标志位
    bool m_convert_flag = false;
};

#endif
