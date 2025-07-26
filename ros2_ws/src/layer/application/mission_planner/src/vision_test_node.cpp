#include "mission_planner/vision_test_node.h"
#include <common_msgs/msg/detail/arm_offboard_status__struct.hpp>
#include <rclcpp/logging.hpp>
#include <utilities/topic_name.hpp>

VisionTestNode::VisionTestNode()
    : rclcpp::Node("vision_test_node")
{
    RCLCPP_INFO(get_logger(), "vision_test_node 节点启动...");
}

void VisionTestNode::initialize(){

    init_pub();
    init_sub();

    m_timer = create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&VisionTestNode::task_loop, this));
}

void VisionTestNode::init_pub(){
    m_pub.arm_offboard_status = create_publisher<common_msgs::msg::ArmOffboardStatus>(
        topic_pub::OFFBOARD_MODE, 10
    );
}

void VisionTestNode::init_sub(){
    m_sub.arm_offboard_status.subscribe(
        shared_from_this(), 
        topic_sub::OFFBOARD_MODE, 10
    );
}

void VisionTestNode::task_loop(){
    switch(m_fly){
        case IDLE:{
            m_interface.mode_control.unlock(
                ARM_ENABLE, ARM_DISABLED, 
                m_sub.arm_offboard_status.get_msg(), 
                m_pub_msgs.arm_offboard_status
            );
            if (m_interface.mode_control.wait_busy()){
                m_fly = TO2Hover;
                RCLCPP_INFO(get_logger(), "初始化完成!");
            }
            break;
        }
        case TO2Hover:{
            
            break;
        }
    }

    m_pub.arm_offboard_status->publish(m_pub_msgs.arm_offboard_status);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionTestNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
