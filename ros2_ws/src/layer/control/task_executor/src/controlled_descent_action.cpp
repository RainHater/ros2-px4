#include "task_executor/controlled_descent_action.h"

ControlledDescentAction::ControlledDescentAction()
    : rclcpp::Node("controlled_descent_action")
{
    RCLCPP_INFO(get_logger(), "controlled_descent_action 节点启动...");
}

void ControlledDescentAction::initialize(){
    init_action();
}

void ControlledDescentAction::init_action(){

}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlledDescentAction>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
