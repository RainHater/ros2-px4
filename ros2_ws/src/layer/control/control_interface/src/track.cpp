#include "control_interface/track.h"
#include <cmath>

Track::Track(){

}

void Track::normal_track(
    identify::msg::YoloDetection detection,
    px4_msgs::msg::TrajectorySetpoint &pub_tra)
{   
    float image_width = detection.image_width;
    float image_height = detection.image_height;
    float lateral_gain = 0.5;

    int cx = (detection.x_min + detection.x_max) / 2;
    int cy = (detection.y_min + detection.y_max) / 2;

    float dx = (cx - image_width/2) / (image_width/2.0f);   // -1 ~ 1
    float dy = (cy - image_height/2) / (image_height/2.0f); // -1 ~ 1

    float yaw_rate = -dx * 1.0f;   // gain 可调
    float vy = -dx * lateral_gain;
    float vz = -dy * 0.5f;

    pub_tra.position[0] = NAN;
    pub_tra.position[1] = NAN;
    pub_tra.position[2] = NAN;
    pub_tra.velocity[0] = 0.0f;
    pub_tra.velocity[1] = vy;
    pub_tra.velocity[2] = vz;
    pub_tra.yaw = NAN;
    pub_tra.yawspeed = yaw_rate;
}
