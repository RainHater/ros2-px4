#ifndef _STATE_ESTIMATOR_NODE_H
#define _STATE_ESTIMATOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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
    void timer_callback();
    void init_subscription();
    void init_service();
    //当前飞机GPS数据回调
    void current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    //GPS坐标转换服务回调
    void handle_gps_to_local(
        const std::shared_ptr<TransformGpsToLocal::Request> request,
        std::shared_ptr<TransformGpsToLocal::Response> response);
    //GPS差值转换为本地坐标
    void convert_gps_to_position();
    //发布目标位置到控制层
    void publish_target_position(double dlat, double dlon, double dz);
    //角度转弧度
    double deg2rad(double deg);
    //GPS → ENU 简易转换函数（参考点 + 当前点 → x, y）
    void gps_to_local(double lat0, double lon0, 
                    double lat, double lon, 
                    double &x, double &y);
private:
    //定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Service<TransformGpsToLocal>::SharedPtr m_gps_to_local_srv;
    //自身经纬度订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_global_position_sub;
    //订阅当前设定值
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_current_setpoing_sub;
    //目标经纬度获取完成标志位
    bool m_reference_initialized = false;
    //起始经纬度坐标
    GeoCoordinate m_reference_gps;
    //当前经纬度
    GeoCoordinate m_current_gps;
    //当前设定值
    px4_msgs::msg::VehicleOdometry m_current_setpoint;
};

#endif
