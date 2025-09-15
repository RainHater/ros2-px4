#ifndef _MOVEMENT_H
#define _MOVEMENT_H

#include <array>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include <px4_ros_com/frame_transforms.h>

#include "common_msgs/msg/arm_offboard_status.hpp"

#include "utilities/topic_tool.hpp"
#include "utilities/topic_name.hpp"
#include "utilities/tf2_tool.hpp"
#include "utilities/geo_tool.hpp"
#include "utilities/log_printf_tool.hpp"

#include "control_interface/mode_control.h"

namespace movement{
namespace nav_move_to_target{
    enum NavMoveToTargetStep{
        IDLE,
        FLY,
    };

    struct NavMoveToTargetInfo{
        std::array<float, 3> target_nav;
        NavMoveToTargetStep state;
    };
}

namespace justmove{
    enum MovementStep{
        IDLE,
        FLY,
    };

    struct JustmoveInfo{
        double dw = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        double dt = 0.0;

        double start_time;
        double total_time;

        std::array<float, 3> start_pos;
        std::array<float, 3> target_pos;

        MovementStep state;
    };
};

namespace move_by_offset{
    enum MoveByOffsetStep{
        IDLE,
        FLY,
    };

    struct MoveByOffsetInfo{
        std::array<float, 3> cal_pos;
        MoveByOffsetStep cur_step;
    };
}

namespace change_height{
    enum ChangeHeightStep{
        IDLE,
        FLY,
    };

    struct ChangeHeightInfo{
        std::array<float, 3> start_pos;
        ChangeHeightStep state;
    };
};

class Movement{
public:
    Movement();

    //飞往目标经纬度
    bool nav_move_to_target(
        geo_tool::GeoCoordinate target_nav,
        geo_tool::GeoCoordinate start_nav,
        std::array<float, 3> cur_pos,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
    //根据局部整体坐标移动
    bool justmove(
        std::array<float, 3> target_pos,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        rclcpp::Time instant_time,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
        bool auto_angle = false,
        float v = 0.1f
    );
    //室外
    bool justmove_outdoor(
        std::array<float, 3> target_pos,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
        bool auto_angle = false
    );
    //根据当前坐标进行移动
    bool move_by_offset(
        std::array<float, 3> target_pos,
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        rclcpp::Time instant_time,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
        float angle = 0.0f,
        float v = 0.1,
        bool outdoor = false
    );
    //起飞高度
    bool change_height(
        std::array<float, 3> cur_pos,
        std::array<float, 4> flo_q,
        rclcpp::Time instant_time,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs,
        float high, 
        float v = 0.1f,
        bool outdoor = false
    );
    //降落
    void land_mode(
        std::array<float, 4> flo_q,
        px4_msgs::msg::TrajectorySetpoint &pub_pos_msgs
    );
private:
    struct YamlInfo{
        float hor_th = 0.9f;
        float ver_th = 0.4f;
        float delta = 0.5f;
        float land_correction = 0.05f;
        float land_start_time = 3.0f;
        float move_time_out = 3.0f;
        float land_th = 0.2f;
    };
private:
    rclcpp::Logger m_log;
    YamlInfo m_yaml;
    justmove::JustmoveInfo m_justmove;
    change_height::ChangeHeightInfo m_change_height;
    nav_move_to_target::NavMoveToTargetInfo m_move_nav;
    move_by_offset::MoveByOffsetInfo m_move_by_offset_info;
};
};

#endif
