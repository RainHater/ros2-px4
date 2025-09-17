#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace log_printf_tool {
inline void printf_log_pos(
    rclcpp::Logger log,
    std::array<float, 3> target_pos, 
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "pub_pos[0]: %f, pub_pos[1]: %f, pub_pos[2]: %f, "
        "cur_pos[0]: %f, cur_pos[1]: %f, cur_pos[2]: %f",
        target_pos[0],
        target_pos[1],
        target_pos[2],
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}

inline void printf_log_cur_pos(
    rclcpp::Logger log,
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "cur_pos[0]: %f, cur_pos[1]: %f, cur_pos[2]: %f",
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}

inline void printf_log_title_pos(
    rclcpp::Logger log,
    std::string title, 
    std::array<float, 3> cur_pos
) {
    RCLCPP_INFO(log, 
        "%s: pos[0]: %f, pos[1]: %f, pos[2]: %f",
        title.c_str(), 
        cur_pos[0],
        cur_pos[1],
        cur_pos[2]
    );
}
}
