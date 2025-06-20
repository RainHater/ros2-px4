cmake_minimum_required(VERSION 3.8)
project(flight_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include 
)

add_executable(arming_offboard_node src/arming_offboard_node.cpp)
ament_target_dependencies(arming_offboard_node 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
)
install(TARGETS arming_offboard_node DESTINATION lib/${PROJECT_NAME})

add_executable(position_ctrl_node src/position_ctrl_node.cpp)
ament_target_dependencies(position_ctrl_node 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
)
install(TARGETS position_ctrl_node DESTINATION lib/${PROJECT_NAME})

add_executable(gps_navigation_node src/gps_navigation_node.cpp)
ament_target_dependencies(gps_navigation_node 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
)
install(TARGETS gps_navigation_node DESTINATION lib/${PROJECT_NAME})

add_executable(velocity_ctrl_node src/velocity_ctrl_node.cpp)
ament_target_dependencies(velocity_ctrl_node 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
)
install(TARGETS velocity_ctrl_node DESTINATION lib/${PROJECT_NAME})

ament_package()
