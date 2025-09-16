#include "control_motion/flight_mode_manager_node.h"

constexpr auto ARM_ENABLE = common_msgs::msg::ArmOffboardStatus::ARM_ENABLE;
constexpr auto ARM_DISABLED = common_msgs::msg::ArmOffboardStatus::ARM_DISABLED;
constexpr auto OFFBOARD_DISABLED = common_msgs::msg::ArmOffboardStatus::OFFBOARD_DISABLED;
constexpr auto ARMING_STATE_ARMED = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
constexpr auto ARMING_STATE_DISARMED = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
constexpr auto NAVIGATION_STATE_OFFBOARD = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
constexpr auto VEHICLE_CMD_NAV_WAYPOINT = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_WAYPOINT;
constexpr auto VEHICLE_CMD_DO_SET_MODE = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
constexpr auto VEHICLE_CMD_MISSION_START = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_MISSION_START;
constexpr auto VEHICLE_CMD_COMPONENT_ARM_DISARM = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
constexpr auto PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
constexpr auto PX4_CUSTOM_MAIN_MODE_MISSION = 3;

using std::placeholders::_1;

FlightModeManagerNode::FlightModeManagerNode() 
    : Node("flight_mode_manager_node")
    , m_flight_state(IDLE)
{   
    RCLCPP_INFO(get_logger(), "flight_mode_manager_node 节点启动...");

    m_offb_mode.cur_arm = ARM_DISABLED;
    m_offb_mode.cur_offb = OFFBOARD_DISABLED;
    m_offb_mode.setpoint_counter = 0;
}

void FlightModeManagerNode::initialize(){
    initPub();
    initSub();

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&FlightModeManagerNode::armAndSetOffboard, this)
    );
}

void FlightModeManagerNode::initPub(){
    auto& topics = utilities::TopicInfo::getInstance();
    
    m_pub.offb_ctrl_mode = create_publisher<px4_msgs::msg::OffboardControlMode>(
        topics.topic_px4_in().OFFBOARD_CONTROL_MODE, 10);

    m_pub.vehicle_cmd = create_publisher<px4_msgs::msg::VehicleCommand>(
        topics.topic_px4_in().VEHICLE_COMMAND, 10);

    m_pub.px4_mode_broad = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topics.topic_out().PX4_MODE, 10);
}

void FlightModeManagerNode::initSub(){
    auto& topics = utilities::TopicInfo::getInstance();
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_sub.batt_status = create_subscription<px4_msgs::msg::BatteryStatus>(
        topics.topic_px4_out().BATTERY_STATUS,
        qos,
        std::bind(&FlightModeManagerNode::battStatusCallback, this, _1)
    );

    m_sub.px4_offb = create_subscription<px4_msgs::msg::VehicleStatus>(
        topics.topic_px4_out().VEHICLE_STATUS,
        qos,
        std::bind(&FlightModeManagerNode::px4OffboardCallback, this, _1)
    );

    m_sub.set_offb = create_subscription<common_msgs::msg::ArmOffboardStatus>(
        topics.topic_in().PX4_MODE,
        10,
        std::bind(&FlightModeManagerNode::setOffboardCallback, this, _1)
    );
}

void FlightModeManagerNode::armAndSetOffboard() {
    pubCurrentMode();

    switch (m_flight_state) {
        case IDLE: {
            m_offb_mode.cur_arm = ARM_DISABLED;
            m_offb_mode.cur_offb = OFFBOARD_DISABLED;       
            if (m_offb_mode.tar_arm == ARM_ENABLE) {
                m_offb_mode.setpoint_counter = 0;
                m_flight_state = SENDING_SETPOINT;
                RCLCPP_INFO(get_logger(), "开始解锁offboard和arm");
            }
            break;
        }

        case SENDING_SETPOINT: {
            pubPx4OffboardMode();
            m_offb_mode.setpoint_counter++;
            if (m_offb_mode.setpoint_counter >= 10) {
                pubVehicleCommand(VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
                m_flight_state = WAITING_OFFBOARD_CONFIRM;
                RCLCPP_INFO(get_logger(), "已发送10次 setpoint");
            }
            break;
        }

        case WAITING_OFFBOARD_CONFIRM: {
            if (m_offb_mode.cur_px4_offb == NAVIGATION_STATE_OFFBOARD) {
                m_flight_state = ARMING;
                RCLCPP_INFO(get_logger(), "已进入 offboard 模式");
            } else {
                // 若未成功进入 Offboard，可尝试重新发送 setpoint
                m_offb_mode.setpoint_counter = 0;
                m_flight_state = SENDING_SETPOINT;
                RCLCPP_INFO(get_logger(), "进入 offboard 模式失败");
            }
            break;
        }

        case ARMING: {
            pubVehicleCommand(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            m_flight_state = WAITING_ARM_CONFIRM;
            break;
        }

        case WAITING_ARM_CONFIRM: {
            if (m_offb_mode.cur_px4_arm == ARMING_STATE_ARMED) {
                m_flight_state = READY;
            }else {
                m_offb_mode.setpoint_counter = 0;
                m_flight_state = SENDING_SETPOINT;
                RCLCPP_INFO(get_logger(), "解锁 arm 失败");
            }
            break;
        }

        case READY: {
            // 可执行飞行任务，或者等待任务触发
            if (m_offb_mode.tar_arm == ARM_DISABLED){
                disarm();
                pubVehicleCommand(VEHICLE_CMD_DO_SET_MODE, 1, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
                m_flight_state = IDLE;
                RCLCPP_INFO(get_logger(), "已上锁 arm");
                return;
            }
            m_offb_mode.cur_arm = m_offb_mode.tar_arm;
            m_offb_mode.cur_offb = m_offb_mode.tar_offb;
            pubPx4OffboardMode();
            break;
        }
        case FAILED: {
            // 错误处理逻辑，可重置状态机等
            break;
        }

        default:
            break;
    }
}

void FlightModeManagerNode::pubPx4OffboardMode() {
    using ArmOffboardStatus = common_msgs::msg::ArmOffboardStatus;
    auto POSITION = ArmOffboardStatus::POSITION;
    auto VELOCITY = ArmOffboardStatus::VELOCITY;
    auto ACCELERATION = ArmOffboardStatus::ACCELERATION;
    auto ATTITUDE = ArmOffboardStatus::ATTITUDE;
    auto BODY_RATE = ArmOffboardStatus::BODY_RATE;
    auto THRUST_AND_TORQUE = ArmOffboardStatus::THRUST_AND_TORQUE;
    auto DIRECT_ACTUATOR = ArmOffboardStatus::DIRECT_ACTUATOR;
    auto mode = m_offb_mode.tar_offb;
    
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = (bool)((mode & POSITION) != 0);
    msg.velocity = (bool)((mode & VELOCITY) != 0);
    msg.acceleration = (bool)((mode & ACCELERATION) != 0);
    msg.attitude = (bool)((mode & ATTITUDE) != 0);
    msg.body_rate = (bool)((mode & BODY_RATE) != 0);
    msg.thrust_and_torque = (bool)((mode & THRUST_AND_TORQUE) != 0);
    msg.direct_actuator = (bool)((mode & DIRECT_ACTUATOR) != 0);
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_pub.offb_ctrl_mode->publish(msg);
}

void FlightModeManagerNode::pubCurrentMode(){
    common_msgs::msg::ArmOffboardStatus msg;
    msg.arm = m_offb_mode.cur_arm;
    msg.offboard = m_offb_mode.cur_offb;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_pub.px4_mode_broad->publish(msg);
}

void FlightModeManagerNode::battStatusCallback(
    const std::shared_ptr<px4_msgs::msg::BatteryStatus> msg
){
    auto remaining = msg->remaining;
    RCLCPP_INFO(
        get_logger(), 
        "当前电量 %.0f %%",
        (remaining * 100.0)  
    );
    m_sub.batt_status.reset();
}

void FlightModeManagerNode::px4OffboardCallback(
    const std::shared_ptr<px4_msgs::msg::VehicleStatus> msg
){
    m_offb_mode.cur_px4_arm = msg->arming_state;
    m_offb_mode.cur_px4_offb = msg->nav_state;
}

void FlightModeManagerNode::setOffboardCallback(
    const std::shared_ptr<common_msgs::msg::ArmOffboardStatus> msg
){
    m_offb_mode.tar_arm = msg->arm;
    m_offb_mode.tar_offb = msg->offboard;
}

void FlightModeManagerNode::pubVehicleCommand(
    uint16_t command,
    float param1,
    float param2,
    float param3,
    float param4,
    float param5,
    float param6,
    float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    m_pub.vehicle_cmd->publish(msg);
}

void FlightModeManagerNode::arm() {
    pubVehicleCommand(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_DEBUG(get_logger(), "Arm command send");
}

void FlightModeManagerNode::disarm() {
    pubVehicleCommand(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_DEBUG(get_logger(), "Disarm command send");
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightModeManagerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
