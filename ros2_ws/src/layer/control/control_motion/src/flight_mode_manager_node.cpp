#include "control_motion/flight_mode_manager_node.h"
#include <cmath>

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

FlightModeManagerNode::FlightModeManagerNode() 
    : Node("flight_mode_manager_node")
{   
    RCLCPP_INFO(get_logger(), "flight_mode_manager_node 节点启动...");

    m_arm_offboard_mode.current.arm = ARM_DISABLED;
    m_arm_offboard_mode.current.offboard = OFFBOARD_DISABLED;
    m_arm_offboard_mode.setpoint_counter = 0;
    
    m_flight_state = IDLE;
}

void FlightModeManagerNode::initialize(){
    init_pub();
    init_sub();

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&FlightModeManagerNode::arm_and_set_offboard, this)
    );
}

void FlightModeManagerNode::init_pub(){
    m_pub.offboard_control_mode = create_publisher<px4_msgs::msg::OffboardControlMode>(
        topic_px4_in::OFFBOARD_CONTROL_MODE, 10);

    m_pub.vehicle_command = create_publisher<px4_msgs::msg::VehicleCommand>(
        topic_px4_in::VEHICLE_COMMAND, 10);

    m_pub.px4_mode_status_broadcaster = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_out::PX4_MODE, 10);
}

void FlightModeManagerNode::init_sub(){
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_arm_offboard_mode.target.subscribe(
        shared_from_this(), 
        topic_in::PX4_MODE, 10
    );

    m_arm_offboard_mode.px4_mode.subscribe(
        shared_from_this(), 
        topic_px4_out::VEHICLE_STATUS, qos
    );

    m_sub.battery_status.subscribe(
        shared_from_this(), 
        topic_px4_out::BATTERY_STATUS, qos
    );
}

void FlightModeManagerNode::arm_and_set_offboard() {
    auto &mode = m_arm_offboard_mode;
    const auto &target_mode = mode.target.get_msg();

    pub_current_mode();

    switch (m_flight_state) {
        case IDLE: {
            m_arm_offboard_mode.current.arm = ARM_DISABLED;
            m_arm_offboard_mode.current.offboard = OFFBOARD_DISABLED;       
            if (!mode.target.has_change()) 
                return;
            if (target_mode.arm == ARM_ENABLE) {
                mode.setpoint_counter = 0;
                m_flight_state = SENDING_SETPOINT;
                RCLCPP_INFO(get_logger(), "开始解锁offboard和arm");
            }
            break;
        }

        case SENDING_SETPOINT: {
            pub_px4_offboard_mode();
            mode.setpoint_counter++;
            if (mode.setpoint_counter >= 10) {
                pub_vehicle_command(VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
                m_flight_state = WAITING_OFFBOARD_CONFIRM;
                RCLCPP_INFO(get_logger(), "已发送10次 setpoint");
            }
            break;
        }

        case WAITING_OFFBOARD_CONFIRM: {
            if (!mode.px4_mode.has_change()) 
                return;
            const auto &px4_mode = mode.px4_mode.get_msg();
            if (px4_mode.nav_state == NAVIGATION_STATE_OFFBOARD) {
                m_flight_state = ARMING;
                RCLCPP_INFO(get_logger(), "已进入 offboard 模式");
            } else {
                // 若未成功进入 Offboard，可尝试重新发送 setpoint
                mode.setpoint_counter = 0;
                m_flight_state = SENDING_SETPOINT;
                RCLCPP_INFO(get_logger(), "进入 offboard 模式失败");
            }
            break;
        }

        case ARMING: {
            pub_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            m_flight_state = WAITING_ARM_CONFIRM;
            break;
        }

        case WAITING_ARM_CONFIRM: {
            if (!mode.px4_mode.has_change()) 
                return;
            const auto &px4_mode = mode.px4_mode.get_msg();
            if (px4_mode.arming_state == ARMING_STATE_ARMED) {
                auto battery_status = m_sub.battery_status.get_first_msg();
                float remaining  = battery_status.remaining;
                m_flight_state = READY;
                RCLCPP_INFO(
                    get_logger(), 
                    "已解锁 arm, 当前电量 %.0f %%",
                    (remaining * 100.0)  
                );
            }else {
                mode.setpoint_counter = 0;
                m_flight_state = SENDING_SETPOINT;
                RCLCPP_INFO(get_logger(), "解锁 arm 失败");
            }
            break;
        }

        case READY: {
            pub_px4_offboard_mode();
            // 可执行飞行任务，或者等待任务触发
            if (!mode.target.has_change()) 
                return;
            if (target_mode.arm == ARM_DISABLED){
                disarm();
                pub_vehicle_command(VEHICLE_CMD_DO_SET_MODE, 1, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
                m_flight_state = IDLE;
                RCLCPP_INFO(get_logger(), "已上锁 arm");
                return;
            }
            mode.current = target_mode;
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

void FlightModeManagerNode::pub_px4_offboard_mode() {
    using ArmOffboardStatus = common_msgs::msg::ArmOffboardStatus;
    auto POSITION = ArmOffboardStatus::POSITION;
    auto VELOCITY = ArmOffboardStatus::VELOCITY;
    auto ACCELERATION = ArmOffboardStatus::ACCELERATION;
    auto ATTITUDE = ArmOffboardStatus::ATTITUDE;
    auto BODY_RATE = ArmOffboardStatus::BODY_RATE;
    auto THRUST_AND_TORQUE = ArmOffboardStatus::THRUST_AND_TORQUE;
    auto DIRECT_ACTUATOR = ArmOffboardStatus::DIRECT_ACTUATOR;
    auto mode = m_arm_offboard_mode.target.get_msg().offboard;
    
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = (mode & POSITION) != 0;
    msg.velocity = (mode & VELOCITY) != 0;
    msg.acceleration = (mode & ACCELERATION) != 0;
    msg.attitude = (mode & ATTITUDE) != 0;
    msg.body_rate = (mode & BODY_RATE) != 0;
    msg.thrust_and_torque = (mode & THRUST_AND_TORQUE) != 0;
    msg.direct_actuator = (mode & DIRECT_ACTUATOR) != 0;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_pub.offboard_control_mode->publish(msg);
}

void FlightModeManagerNode::pub_current_mode(){
    auto msg = m_arm_offboard_mode.current;
    m_pub.px4_mode_status_broadcaster->publish(msg);
}

void FlightModeManagerNode::pub_vehicle_command(
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
    m_pub.vehicle_command->publish(msg);
}

void FlightModeManagerNode::arm() {
    pub_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_DEBUG(get_logger(), "Arm command send");
}

void FlightModeManagerNode::disarm() {
    pub_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_DEBUG(get_logger(), "Disarm command send");
}
