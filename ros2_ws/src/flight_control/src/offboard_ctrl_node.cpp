#include "flight_control/offboard_ctrl_node.h"
#include "utilities/util_topic.hpp"
#include <cmath>
#include <functional>
#include <rclcpp/logging.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

static constexpr float kInvalidThreshold = 1e-6f;

OffboardCtrlNode::OffboardCtrlNode() 
    : Node("offboard_ctrl_node") {
    RCLCPP_INFO(get_logger(), "Starting offboard_ctrl_node follower node...");
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    
    init_client();
    m_action_nav_server = rclcpp_action::create_server<NavigateToGPS>(
        this,
        "/control/navigate_to_gps", 
        std::bind(&OffboardCtrlNode::nav_handle_goal, this, _1, _2),
        std::bind(&OffboardCtrlNode::nav_handle_cancel, this, _1),
        std::bind(&OffboardCtrlNode::nav_handle_accepted, this, _1));

    m_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/interface/in/trajectory_setpoint", 10);
    
    m_target_setpoint_sub = create_subscription<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10, 
        std::bind(&OffboardCtrlNode::target_setpoint_callback, this, _1));
    m_current_offboard_mode_sub = utils::make_simple_subscription<
        common_msgs::msg::ArmOffboardStatus>(
      "/control/px4_mode_status_broadcaster",
        qos, this, px4_mode_status_broadcaster);

    m_timer = create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&OffboardCtrlNode::timer_callback, this));
}

void OffboardCtrlNode::init_client(){
    m_gps_transform_client = create_client<TransformGpsToLocal>(
        "/transform_gps_to_local");
}

void OffboardCtrlNode::timer_callback(){
    publish_trajectory_setpoint();
}

void OffboardCtrlNode::target_setpoint_callback(const common_msgs::msg::TrajectorySetPoint::SharedPtr msg){
    auto &mode = px4_mode_status_broadcaster.offboard_mode;
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

rclcpp_action::GoalResponse OffboardCtrlNode::nav_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToGPS::Goal> goal){
        
    RCLCPP_INFO(get_logger(), "Received goal: lat=%f lon=%f alt=%f", goal->lat, goal->lon, goal->alt);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardCtrlNode::nav_handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle){
    
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
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
    auto request = std::make_shared<TransformGpsToLocal::Request>();

    while(true){
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Cancelled";
            goal_handle->canceled(result);
            return;
        }

        request->latitude = goal->lat;
        request->longitude = goal->lon;
        request->altitude = goal->alt;
        if (!m_gps_transform_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "GPS transform service not available.");
            auto result = std::make_shared<NavigateToGPS::Result>();
            result->success = false;
            result->message = "GPS transform service not available.";
            goal_handle->abort(result);
            break;
        }

        auto future = m_gps_transform_client->async_send_request(request,
        [this, goal_handle](rclcpp::Client<TransformGpsToLocal>::SharedFuture future) {
            auto response = future.get();

            m_target_setpoint.position[0] = static_cast<float>(response->x);
            m_target_setpoint.position[1] = static_cast<float>(response->y);
            m_target_setpoint.position[2] = static_cast<float>(response->z);

            // RCLCPP_INFO(this->get_logger(),
            //             "Transformed target to local: x=%.2f, y=%.2f, z=%.2f",
            //             m_target_setpoint.position[0],
            //             m_target_setpoint.position[1],
            //             m_target_setpoint.position[2]);
        });
        
        feedback->current_latitude = 0;
        feedback->current_longitude = 0;
        feedback->current_altitude = 0;
        feedback->distance_remaining = 0;

        goal_handle->publish_feedback(feedback);
    }

    result->success = true;
    result->message = "Arrived at target";
    goal_handle->succeed(result);
}

bool OffboardCtrlNode::request_local_target(double lat, double lon, double alt){
    // 构造请求
    auto request = std::make_shared<TransformGpsToLocal::Request>();
    request->latitude = lat;
    request->longitude = lon;
    request->altitude = alt;

    // 等待服务可用（1秒）
    if (!m_gps_transform_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "GPS transform service not available.");
        return false;
    }

    // 发送异步请求，并等待结果（同步处理）
    auto future = m_gps_transform_client->async_send_request(request);

    // 阻塞等待响应
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call /transform_gps_to_local service.");
        return false;
    }

    // 获取响应
    auto response = future.get();
    m_target_local_position[0] = static_cast<float>(response->x);
    m_target_local_position[1] = static_cast<float>(response->y);
    m_target_local_position[2] = static_cast<float>(response->z);

    RCLCPP_INFO(get_logger(), 
                "Transformed target to local: x=%.2f, y=%.2f, z=%.2f",
                m_target_local_position[0],
                m_target_local_position[1],
                m_target_local_position[2]);

    return true;
}

void OffboardCtrlNode::publish_trajectory_setpoint() {
    const auto &target = m_target_setpoint;
    auto &msg = m_traj_msg_cache;
    auto &POSITION = common_msgs::msg::ArmOffboardStatus::POSITION;
    auto &VELOCITY = common_msgs::msg::ArmOffboardStatus::VELOCITY;
    auto &mode = px4_mode_status_broadcaster.offboard_mode;

    if (mode == POSITION && non_zero3(target.position)){
        utils::copy_float_data(target.position, msg.position);
        msg.yaw = target.yaw;
        msg.yawspeed = 0.0f;
    }else if (mode == VELOCITY && non_zero3(target.velocity)){
        utils::copy_float_data(target.velocity, msg.velocity);
        msg.yaw = NAN;
        msg.yawspeed = target.yawspeed;
    }
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectory_setpoint_pub->publish(msg);
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
