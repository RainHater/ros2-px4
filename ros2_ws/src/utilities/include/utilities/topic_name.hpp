#pragma once

//外部话题的话题订阅

namespace common_topics {
    
}

namespace topic_out {
    using namespace common_topics;
    constexpr const char* PX4_MODE                      = "/control/out/offboard_mode";
    constexpr const char* TRACKING_FEEDBACK             = "/vision_pipeline/tracking_feedback";
    constexpr const char* YOLO_DETECTIONS               = "/yolo_detections";
}

namespace topic_in {
    using namespace common_topics;
    constexpr const char* PX4_MODE                      = "/control/in/offboard_mode";
}

namespace topic_px4_in {
    constexpr const char* VEHICLE_COMMAND               = "/fmu/in/vehicle_command";
    constexpr const char* OFFBOARD_CONTROL_MODE         = "/fmu/in/offboard_control_mode";
    constexpr const char* PX4_TRAJECTORY_SETPOINT       = "/fmu/in/trajectory_setpoint";
    constexpr const char* MISSION                       = "/fmu/in/mission";
    constexpr const char* NAVIGATOR_MISSION_ITEM        = "/fmu/in/navigator_mission_item";
}

namespace topic_px4_out {
    constexpr const char* VEHICLE_STATUS                = "/fmu/out/vehicle_status";
    constexpr const char* VEHICLE_LOCAL_POSITION        = "/fmu/out/vehicle_local_position";
    constexpr const char* VEHICLE_ODOMETRY              = "/fmu/out/vehicle_odometry";
    constexpr const char* VEHICLE_GLOBAL_POSITION       = "/fmu/out/vehicle_global_position";
    constexpr const char* BATTERY_STATUS                = "/fmu/out/battery_status";
    constexpr const char* VEHICLE_ATTITUDE              = "/fmu/out/vehicle_attitude";
    constexpr const char* MANUAL_CONTROL_SETPOINT       = "/fmu/out/manual_control_setpoint";
}
