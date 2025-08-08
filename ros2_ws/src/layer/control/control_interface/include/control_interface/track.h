#ifndef _TRACK_H
#define _TRACK_H

#include <px4_msgs/msg/detail/trajectory_setpoint__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "identify/msg/yolo_detection.hpp"

class Track{
public:
    Track();

    void normal_track(
        identify::msg::YoloDetection detection,
        px4_msgs::msg::TrajectorySetpoint &pub_tra
    );
private:
};

#endif
