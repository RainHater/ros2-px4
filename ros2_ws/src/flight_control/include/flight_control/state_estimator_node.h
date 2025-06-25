#ifndef _STATE_ESTIMATOR_NODE_H
#define _STATE_ESTIMATOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <common_msgs/msg/target_gps.hpp>
#include <common_msgs/msg/trajectory_set_point.hpp>
#include <common_msgs/srv/transform_gps_to_local.hpp>

using common_msgs::srv::TransformGpsToLocal;

struct GeoCoordinate {
    double lat;     // 纬度
    double lon;     // 经度
    double alt;     // 高度
};

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode();
protected:
    void init_service();
    //定时器回调函数
    void timer_callback();
    //自身经纬度回调函数
    void current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    //目标经纬度回调函数
    void target_gps_callback(const common_msgs::msg::TargetGps &msg);
    void handle_gps_to_local(
        const std::shared_ptr<TransformGpsToLocal::Request> request,
        std::shared_ptr<TransformGpsToLocal::Response> response);
    //经纬度转换
    void convert_gps_to_position();
    //发布位置
    void publish_target_position(double dlat, double dlon, double dz);
    double deg2rad(double deg);
    void gps_to_local(double lat0, double lon0, double lat, double lon, double &x, double &y);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Service<TransformGpsToLocal>::SharedPtr m_gps_to_local_srv;
    //发布位置
    rclcpp::Publisher<common_msgs::msg::TrajectorySetPoint>::SharedPtr m_trajectory_setpoint_pub;
    //自身经纬度订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_global_position_sub;
    //目标经纬度订阅
    rclcpp::Subscription<common_msgs::msg::TargetGps>::SharedPtr m_target_gps_sub;
    //目标经纬度消息
    common_msgs::msg::TargetGps m_target_gps;
    //当前经纬度消息
    common_msgs::msg::TargetGps m_current_gps;
    //常量
    const double m_R = 6'371'000.0;
    //起点海拔
    double m_home_alt;
    //目标经纬度获取完成标志位
    bool m_get_target_gps_finish = false;
    //当前经纬度获取完成标志位
    bool m_get_current_gps_finish = false;
    bool m_reference_initialized = false;
    GeoCoordinate m_reference_gps;
};

#endif
