#include "control/gps_nav_node.h"

using std::placeholders::_1;
using std::placeholders::_2;

GpsNavNode::GpsNavNode()
    : Node("gps_nav_node"){
    RCLCPP_INFO(get_logger(), "Starting gps_nav_node follower node...");

    init_publisher();
    init_action();
    init_client();
}

void GpsNavNode::init_publisher(){
    m_target_setpoint_pub = create_publisher<common_msgs::msg::TrajectorySetPoint>(
        "/control/trajectory_setpoint", 10);
}

void GpsNavNode::init_action(){

    m_action_nav_server = rclcpp_action::create_server<CommonNavigateToGPS>(
        this,
        "/control/navigate_to_gps", 
        [this](const rclcpp_action::GoalUUID & uuid, 
        //处理导航目标请求
        std::shared_ptr<const CommonNavigateToGPS::Goal> goal){
            std::stringstream ss;
            for (auto byte : uuid) {
                ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte;
            }
            RCLCPP_INFO(get_logger(), "uuid: %s, nav_handle_goal: lat=%f lon=%f alt=%f", 
                                    ss.str().c_str(), goal->lat, goal->lon, goal->alt);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        //处理取消导航请求
        [this](const std::shared_ptr<GoalHandleNavigate> goal_handle){
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "received request to cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        //接收并准备执行导航任务
        [this](const std::shared_ptr<GoalHandleNavigate> goal_handle){
            std::thread{std::bind(&GpsNavNode::nav_execute, this, goal_handle)}.detach();
        });
}

void GpsNavNode::init_client(){
    m_gps_transform_client = create_client<CommonSrvTransformGpsToLocal>(
        "/perception/transform_gps_to_local");
}

void GpsNavNode::nav_execute(const std::shared_ptr<GoalHandleNavigate> goal_handle){

    auto result = std::make_shared<CommonNavigateToGPS::Result>();
    auto feedback = std::make_shared<CommonNavigateToGPS::Feedback>();
    auto goal = goal_handle->get_goal();

    while(true){
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Cancelled";
            goal_handle->canceled(result);
            return;
        }

        auto request = std::make_shared<CommonSrvTransformGpsToLocal::Request>();
        auto response = std::make_shared<CommonSrvTransformGpsToLocal::Response>();
        request->latitude = goal->lat;
        request->longitude = goal->lon;
        request->altitude = goal->alt;
        if (!request_local_target(request, response))
            continue;
        
        common_msgs::msg::TrajectorySetPoint msg{};

        msg.position[0] = static_cast<float>(response->x);
        msg.position[1] = static_cast<float>(response->y);
        msg.position[2] = static_cast<float>(response->z);
        m_target_setpoint_pub->publish(msg);
        // m_target_setpoint.position[0] = static_cast<float>(response->x);
        // m_target_setpoint.position[1] = static_cast<float>(response->y);
        // m_target_setpoint.position[2] = static_cast<float>(response->z);

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

bool GpsNavNode::request_local_target(
        const std::shared_ptr<CommonSrvTransformGpsToLocal::Request> request, 
        std::shared_ptr<CommonSrvTransformGpsToLocal::Response> &response){

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
    rclcpp::spin(std::make_shared<GpsNavNode>());

    rclcpp::shutdown();
    return 0;
}
