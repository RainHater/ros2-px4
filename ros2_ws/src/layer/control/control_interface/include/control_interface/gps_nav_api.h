#ifndef _GPS_NAV_API_H
#define _GPS_NAV_API_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/navigate_to_gps.hpp"

#include <mutex>

class NavApi{
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
public:
    NavApi();
    static NavApi& Instance();
    void send_goal(const rclcpp::Node::SharedPtr &node,
        double lat, double lon, double alt, std::function<void()> succeeded_callback=nullptr);
private:
    //导航任务忙碌
    bool m_is_busy;
    std::mutex m_mutex;
    rclcpp_action::Client<NavigateToGPS>::SharedPtr m_client;
};

#endif
