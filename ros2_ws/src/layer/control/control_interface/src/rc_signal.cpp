#include "control_interface/rc_signal.h"

RcSignal::RcSignal()
 : m_log(rclcpp::get_logger("RC遥控信号(rc_signal.cpp)"))
{

}

rc_signal::RcMode RcSignal::get_rc(px4_msgs::msg::ManualControlSetpoint sub_rc){

    auto rc_mode = rc_signal::NONE;

    if (sub_rc.aux1 < 0){
        rc_mode = rc_signal::LAND;
    }

    return rc_mode;
}
