#pragma once

#include <rclcpp/rclcpp.hpp>

namespace log_printf_tool {

inline void printf_log_pos(
    rclcpp::Logger log,
    std::array<float, 3> target_pos, 
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "position[0]: %f, position[1]: %f, position[2]: %f, "
        "current[0]: %f, current[1]: %f, current[2]: %f",
        target_pos[0],
        target_pos[1],
        target_pos[2],
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}
}
