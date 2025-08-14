#ifndef _RC_SIGNAL_H
#define _RC_SIGNAL_H

#include <px4_msgs/msg/detail/manual_control_setpoint__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>

namespace rc_signal{
enum RcMode{
    NONE = 0,
    LAND,
};
};

class RcSignal{
public:
    RcSignal();
    rc_signal::RcMode get_rc(px4_msgs::msg::ManualControlSetpoint sub_rc);
private:
    rclcpp::Logger m_log;
};

#endif
