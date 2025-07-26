#include "control_motion/flight_mode_manager_node.h"
#include "control_motion/offboard_ctrl_node.h"

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // auto offboard_node = std::make_shared<OffboardCtrlNode>();
    auto flight_mode_node = std::make_shared<FlightModeManagerNode>();

    // offboard_node->initialize();
    flight_mode_node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(offboard_node);
    executor.add_node(flight_mode_node);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
