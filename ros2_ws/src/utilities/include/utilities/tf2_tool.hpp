#pragma once

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace tf2_tool {
struct EulerAngles {
    double roll;
    double pitch;
    double yaw;
};
//获取角度
inline void get_euler_angles(
    px4_msgs::msg::VehicleOdometry setpoint, 
    EulerAngles &angles)
{
    tf2::Quaternion q(
        setpoint.q[1],  // x
        setpoint.q[2],  // y
        setpoint.q[3],  // z
        setpoint.q[0]   // w
    );
    tf2::Matrix3x3(q).getRPY(angles.roll, angles.pitch, angles.yaw);
}
}
