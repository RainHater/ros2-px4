#include "control_interface/track.h"
#include <cmath>

Track::Track(){

}

// void Track::normal_track(
//     identify::msg::YoloDetection detection,
//     px4_msgs::msg::VehicleOdometry sub_pose,
//     px4_msgs::msg::TrajectorySetpoint &pub_tra)
// {   
//     float image_width = detection.image_width;

//     float dx = (detection.cx - image_width / 2);

//     float yaw_angle = -dx * M_PI;  // 映射偏航角，-1~1 映射到 -π ~ π ，可根据需求调整增益和范围
//     tf2_tool::EulerAngles angles;
//     tf2_tool::get_euler_angles(sub_pose, angles);
//     // 只设置目标yaw角
//     // pub_tra.position[0] = NAN;
//     // pub_tra.position[1] = NAN;
//     // pub_tra.position[2] = NAN;

//     // pub_tra.velocity[0] = 0.0f;
//     // pub_tra.velocity[1] = 0.0f;
//     // pub_tra.velocity[2] = 1;

//     // pub_tra.yawspeed = 0.0f;

//     pub_tra.yaw = yaw_angle;
//     RCLCPP_INFO(rclcpp::get_logger(
//         "跟踪"), 
//         "yaw_angle: %f",
//         yaw_angle
//     );
// }

void Track::normal_track(
    identify::msg::YoloDetection detection,
    px4_msgs::msg::VehicleOdometry sub_pose,
    px4_msgs::msg::TrajectorySetpoint &pub_tra)
{       
    float kp = 0.0015;
    float ki = 0.00001;
    float image_width = detection.image_width;
    float error = (detection.cx - image_width / 2);

    m_integral += error;

    float p_term = kp * error;
    float i_term = ki * m_integral;
    
    pub_tra.yawspeed = p_term + i_term;

    RCLCPP_INFO(rclcpp::get_logger(
        "跟踪"), 
        "yawspeed: %f",
        pub_tra.yawspeed
    );
}

