#ifndef _TRACK_H
#define _TRACK_H

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "identify/msg/yolo_detection.hpp"

#include "utilities/tf2_tool.hpp"

class Track{
public:
    Track();

    void normal_track(
        identify::msg::YoloDetection detection,
        px4_msgs::msg::VehicleOdometry sub_pose,
        px4_msgs::msg::TrajectorySetpoint &pub_tra
    );
private:
    float m_integral;
};

#endif
