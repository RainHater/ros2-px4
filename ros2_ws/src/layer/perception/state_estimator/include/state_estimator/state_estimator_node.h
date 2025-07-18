#ifndef _STATE_ESTIMATOR_NODE_H
#define _STATE_ESTIMATOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <common_msgs/srv/transform_gps_to_local.hpp>

#include "utilities/topic_tool.hpp"
#include "utilities/geo_tool.hpp"

using common_msgs::srv::TransformGpsToLocal;

struct GeoReferenceStatus {
    //目标经纬度获取完成标志位
    bool reference_initialized = false;
    //起始经纬度坐标
    geo_tool::GeoCoordinate reference_gps;
    //当前经纬度
    geo_tool::GeoCoordinate current_gps;
};

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode();
    void initialized();
protected:
    void init_subscription();
    void init_service();

    //当前飞机GPS数据回调
    void current_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    //GPS坐标转换服务回调
    void handle_gps_to_local(
        const std::shared_ptr<TransformGpsToLocal::Request> request,
        std::shared_ptr<TransformGpsToLocal::Response> response
    );
    //发布目标位置到控制层
    void publish_target_position(double dlat, double dlon, double dz);
private:
    //GPS坐标转换服务
    rclcpp::Service<TransformGpsToLocal>::SharedPtr m_gps_to_local_srv;
    //自身经纬度订阅
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_global_position_sub;
    
    //消息监听
    TopicListener<px4_msgs::msg::VehicleOdometry> m_current_setpoing_listener;
    
    //地理参考状态
    GeoReferenceStatus m_geo_ref_status;
};

#endif
