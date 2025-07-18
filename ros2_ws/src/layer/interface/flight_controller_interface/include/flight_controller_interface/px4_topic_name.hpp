#pragma once

namespace topic_px4_out {
    constexpr const char* VEHICLE_ODOMETRY              = "/fmu/out/vehicle_odometry";
    constexpr const char* VEHICLE_STATUS                = "/fmu/out/vehicle_status";
    constexpr const char* VEHICLE_GLOBAL_POSITION       = "/fmu/out/vehicle_global_position";
    constexpr const char* VEHICLE_LOCAL_POSITION        = "/fmu/out/vehicle_local_position";
}

namespace topic_px4_in {
    constexpr const char* VEHICLE_COMMAND               = "/fmu/in/vehicle_command";
    constexpr const char* TRAJECTORY_SETPOINT           = "/fmu/in/trajectory_setpoint";
    constexpr const char* OFFBOARD_CONTROL_MODE         = "/fmu/in/offboard_control_mode";
}
