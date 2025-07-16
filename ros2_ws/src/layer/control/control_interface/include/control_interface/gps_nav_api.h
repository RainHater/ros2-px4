#ifndef _GPS_NAV_API_H
#define _GPS_NAV_API_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/navigate_to_gps.hpp"

class NavApi{
public:
    using NavigateToGPS = common_msgs::action::NavigateToGPS;
public:
    NavApi();
    static NavApi& Instance();
    void send_goal(const rclcpp::Node::SharedPtr &node,
        rclcpp_action::Client<NavigateToGPS>::SharedPtr &nav_clinet, 
        double lat, double lon, double alt, std::function<void()> succeeded_callback=nullptr);
private:
    //导航任务忙碌
    bool m_nav_is_busy;
};

#endif
