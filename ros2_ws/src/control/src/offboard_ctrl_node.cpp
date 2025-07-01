#include "control/offboard_ctrl_node.h"
#include "utilities/util_topic.hpp"
#include <cmath>
#include <functional>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <iomanip>

using std::placeholders::_1;
using std::placeholders::_2;

static constexpr float kInvalidThreshold = 1e-6f;

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting offboard_ctrl_node follower node...");
    
    init_publisher();
    init_subscription();
    init_action();
    init_client();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&OffboardCtrlNode::timer_callback, this));
}

void OffboardCtrlNode::init_publisher(){
    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10);
}

void OffboardCtrlNode::init_subscription(){
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_target_setpoint_sub = create_subscription<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10, 
        std::bind(&OffboardCtrlNode::target_setpoint_callback, this, _1));
    m_current_offboard_mode_sub = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        "/control/px4_mode_status_broadcaster", qos, 
        std::bind(&OffboardCtrlNode::current_offboard_mode_callback, this, _1));
}

void OffboardCtrlNode::init_action(){
    m_action_nav_server = rclcpp_action::create_server<NavigateToGPS>(
        this,
        "/control/navigate_to_gps", 
        std::bind(&OffboardCtrlNode::nav_handle_goal, this, _1, _2),
        std::bind(&OffboardCtrlNode::nav_handle_cancel, this, _1),
        std::bind(&OffboardCtrlNode::nav_handle_accepted, this, _1));
}

void OffboardCtrlNode::init_client(){
    m_gps_transform_client = create_client<TransformGpsToLocal>(
        "/perception/transform_gps_to_local");
}

void OffboardCtrlNode::timer_callback(){
    publish_trajectory_setpoint();
}

void OffboardCtrlNode::target_setpoint_callback(const common_msgs::msg::TrajectorySetPoint::SharedPtr msg){
    auto &mode = m_current_offboard_mode.offboard_mode;
    auto &POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
    auto &VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;

    if (mode == POSITION){
        utils::copy_float_data(msg->position, m_target_setpoint.position);
    }else if (mode == VELOCITY){
        utils::copy_float_data(msg->velocity, m_target_setpoint.velocity);
    }
    m_target_setpoint.yaw = msg->yaw;
    m_target_setpoint.yawspeed = msg->yawspeed;
}

void OffboardCtrlNode::current_offboard_mode_callback(const common_msgs::msg::ArmOffboardStatus::SharedPtr msg){
    if (m_current_offboard_mode.offboard_mode != msg->offboard_mode){
        m_current_offboard_mode.offboard_mode = msg->offboard_mode;
        m_target_setpoint.position.fill(0.0f);
        m_target_setpoint.velocity.fill(0.0f);
        m_target_setpoint.yaw = 0.0f;
        m_target_setpoint.yawspeed = 0.0f;
    }
    m_current_offboard_mode.arming_state = msg->arming_state;
}

rclcpp_action::GoalResponse OffboardCtrlNode::nav_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToGPS::Goal> goal){
    std::stringstream ss;
    for (auto byte : uuid) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte;
    }
    RCLCPP_INFO(get_logger(), "uuid: %s, nav_handle_goal: lat=%f lon=%f alt=%f", ss.str().c_str(), goal->lat, goal->lon, goal->alt);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardCtrlNode::nav_handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle){
    
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardCtrlNode::nav_handle_accepted(
        const std::shared_ptr<GoalHandleNavigate> goal_handle){
    
    std::thread{std::bind(&OffboardCtrlNode::nav_execute, this, goal_handle)}.detach();
}

void OffboardCtrlNode::nav_execute(
        const std::shared_ptr<GoalHandleNavigate> goal_handle){

    auto result = std::make_shared<NavigateToGPS::Result>();
    auto feedback = std::make_shared<NavigateToGPS::Feedback>();
    auto goal = goal_handle->get_goal();

    while(true){
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Cancelled";
            goal_handle->canceled(result);
            return;
        }

        auto request = std::make_shared<TransformGpsToLocal::Request>();
        auto response = std::make_shared<TransformGpsToLocal::Response>();
        request->latitude = goal->lat;
        request->longitude = goal->lon;
        request->altitude = goal->alt;
        if (!request_local_target(request, response))
            continue;
        
        m_target_setpoint.position[0] = static_cast<float>(response->x);
        m_target_setpoint.position[1] = static_cast<float>(response->y);
        m_target_setpoint.position[2] = static_cast<float>(response->z);

        feedback->current_latitude = response->lat;
        feedback->current_longitude = response->lon;
        feedback->current_altitude = response->alt;
        feedback->distance_remaining = 0;

        goal_handle->publish_feedback(feedback);

        if (response->arrive)
            break;
    }

    result->success = true;
    result->message = "Arrived at target";
    goal_handle->succeed(result);
}

bool OffboardCtrlNode::request_local_target(
        const std::shared_ptr<TransformGpsToLocal::Request> request,
        std::shared_ptr<TransformGpsToLocal::Response> &response){

    if (!m_gps_transform_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "GPS transform service not available.");
        return false;
    }

    auto future = m_gps_transform_client->async_send_request(request);
    response = future.get();
    return true;
}

void OffboardCtrlNode::publish_trajectory_setpoint() {
    const auto &target = m_target_setpoint;
    const auto &arm_state = m_current_offboard_mode.arming_state;
    const auto &mode = m_current_offboard_mode.offboard_mode;
    const auto &armed = common_msgs::msg::ArmOffboardStatus::ARMING_STATE_ARMED;
    const auto &POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
    const auto &VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
    if (arm_state != armed)
        return;

    px4_msgs::msg::TrajectorySetpoint msg{};   
    auto use_if_mode = [&](auto& target_mode, float value) {
        return (mode == target_mode) ? value : NAN;
    };
    
    msg.position[0] = use_if_mode(POSITION, target.position[0]);
    msg.position[1] = use_if_mode(POSITION, target.position[1]);
    msg.position[2] = use_if_mode(POSITION, target.position[2]);
    msg.velocity[0] = use_if_mode(VELOCITY, target.velocity[0]);
    msg.velocity[1] = use_if_mode(VELOCITY, target.velocity[1]);
    msg.velocity[2] = use_if_mode(VELOCITY, target.velocity[2]);
    msg.yaw = use_if_mode(POSITION, target.yaw);
    msg.yawspeed = use_if_mode(VELOCITY, target.yawspeed);
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectory_setpoint_pub->publish(msg);

    if (mode == POSITION){
        RCLCPP_INFO(get_logger(), "Position Setpoint: px=%.2f, py=%.2f, pz=%.2f, yaw=%.2f",
            msg.position[0], msg.position[1], msg.position[2], msg.yaw);
    }else if (mode == VELOCITY){
        RCLCPP_INFO(get_logger(), "Velocity Setpoint: vx=%.2f, vy=%.2f, vz=%.2f, yawspeed=%.2f",
            msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawspeed);
    }   
}

bool OffboardCtrlNode::non_zero3(const std::array<float, 3>& v){
    return  std::fabs(v[0]) > kInvalidThreshold ||
            std::fabs(v[1]) > kInvalidThreshold ||
            std::fabs(v[2]) > kInvalidThreshold;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardCtrlNode>());

    rclcpp::shutdown();
    return 0;
}
