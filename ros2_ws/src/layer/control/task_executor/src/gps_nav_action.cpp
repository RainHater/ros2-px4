#include "task_executor/gps_nav_action.h"
#include "utilities/topic_name.hpp"

GpsNavAction::GpsNavAction()
    : Node("gps_nav_action")
{
    RCLCPP_INFO(get_logger(), "Starting gps_nav_action follower node...");
}

void GpsNavAction::initialize(){
    init_publisher();
    init_action();
    init_client();
}

void GpsNavAction::init_publisher(){
    m_target_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        topic_pub::TRAJECTORY_SETPOINT, 10);
}

void GpsNavAction::init_action(){
    m_action_srv = rclcpp_action::create_server<NavigateToGPS>(
        this,
        topic_srv::NAVIGATE_TO_GPS, 
        //处理导航目标请求
        [this](const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const NavigateToGPS::Goal> goal){
            m_uuid = rclcpp_action::to_string(uuid);
            RCLCPP_INFO(get_logger(), "任务id: %s, 接收的目标航点: lat=%f lon=%f alt=%f", 
                                    m_uuid.c_str(), 
                                    goal->lat, goal->lon, goal->alt);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        //处理取消导航请求
        [this](const std::shared_ptr<GoalHandleNavigate> goal_handle){
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "任务id: %s, 目标航点已取消", m_uuid.c_str());
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        //接收并准备执行导航任务
        [this](const std::shared_ptr<GoalHandleNavigate> goal_handle){
            std::thread{std::bind(&GpsNavAction::execute, this, goal_handle)}.detach();
        });
}

void GpsNavAction::init_client(){
    m_gps_transform_client = create_client<SrvTransformGpsToLocal>(
        topic_cli::TRANSFORM_GPS_TO_LOCAL);
}

void GpsNavAction::execute(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
    auto result = std::make_shared<NavigateToGPS::Result>();
    auto feedback = std::make_shared<NavigateToGPS::Feedback>();
    auto goal = goal_handle->get_goal();

    RCLCPP_INFO(get_logger(), "任务id: %s 开始执行", m_uuid.c_str());

    while(rclcpp::ok()){
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Cancelled";
            goal_handle->canceled(result);
            return;
        }

        auto request = std::make_shared<SrvTransformGpsToLocal::Request>();
        auto response = std::make_shared<SrvTransformGpsToLocal::Response>();
        request->latitude = goal->lat;
        request->longitude = goal->lon;
        request->altitude = goal->alt;
        if (!request_local_target(request, response))
            continue;
        
        common_msgs::msg::TrajectorySetPoint msg{};

        msg.position[0] = static_cast<float>(response->x);
        msg.position[1] = static_cast<float>(response->y);
        msg.position[2] = static_cast<float>(response->z);
        msg.yaw = static_cast<float>(response->yaw);
        m_target_setpoint_pub->publish(msg);

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
    RCLCPP_INFO(get_logger(), "任务id: %s 已完成", m_uuid.c_str());
}

bool GpsNavAction::request_local_target(
    const std::shared_ptr<SrvTransformGpsToLocal::Request> request, 
    std::shared_ptr<SrvTransformGpsToLocal::Response> &response)
{
    if (!m_gps_transform_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "GPS transform service not available.");
        return false;
    }

    auto future = m_gps_transform_client->async_send_request(request);
    response = future.get();
    return true;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsNavAction>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
