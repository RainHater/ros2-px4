#pragma once

//外部话题的话题订阅

namespace common_topics {
    constexpr const char* TRAJECTORY_SETPOINT       = "/control/trajectory_setpoint";
    constexpr const char* PX4_MODE_STATUS           = "/control/px4_mode_status_broadcaster";
    constexpr const char* SET_OFFBOARD_MODE         = "/control/set_offboard_mode";
}

namespace topic_sub {
    using namespace common_topics;
    constexpr const char* VEHICLE_STATUS                = "/interface/out/vehicle_status";
    constexpr const char* VEHICLE_LOCAL_POSITION        = "/interface/out/vehicle_local_position";
    constexpr const char* VEHICLE_ODOMETRY              = "/interface/out/vehicle_odometry";
    constexpr const char* VEHICLE_GLOBAL_POSITION       = "/interface/out/vehicle_global_position";
    constexpr const char* TRACKING_FEEDBACK             = "/vision_pipeline/tracking_feedback";
}

namespace topic_pub {
    using namespace common_topics;
    constexpr const char* VEHICLE_COMMAND               = "/interface/in/vehicle_command";
    constexpr const char* OFFBOARD_CONTROL_MODE         = "/interface/in/offboard_control_mode";
    constexpr const char* PX4_TRAJECTORY_SETPOINT       = "/interface/in/trajectory_setpoint";
    constexpr const char* PID_VIEWER                    = "/debug/pid_viewer";
}

namespace common_services {
    constexpr const char* TRANSFORM_GPS_TO_LOCAL        = "/perception/transform_gps_to_local";
    constexpr const char* NAVIGATE_TO_GPS               = "/control/navigate_to_gps";
    constexpr const char* FLY_RELATIVE_DIRECTION        = "/control/fly_relative_direction";
}

namespace topic_srv {
    using namespace common_services;
}

namespace topic_cli {
    using namespace common_services;
}
