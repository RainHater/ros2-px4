cmake_minimum_required(VERSION 3.8)
project(task_executor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(utilities REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_interface REQUIRED)

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

add_executable(gps_nav_action src/gps_nav_action.cpp)
ament_target_dependencies(gps_nav_action 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
  utilities
  rclcpp_action
)
install(TARGETS gps_nav_action DESTINATION lib/${PROJECT_NAME})

include_directories(
  ${PROJECT_SOURCE_DIR}/include 
)

add_executable(fly_relative_direction_action src/fly_relative_direction_action.cpp)
ament_target_dependencies(fly_relative_direction_action 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
  utilities
  rclcpp_action
  control_interface
)
install(TARGETS fly_relative_direction_action DESTINATION lib/${PROJECT_NAME})

add_executable(controlled_descent_action src/controlled_descent_action.cpp)
ament_target_dependencies(controlled_descent_action 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
  utilities
  rclcpp_action
  control_interface
)
install(TARGETS controlled_descent_action DESTINATION lib/${PROJECT_NAME})

add_executable(set_offboard_mode_action src/set_offboard_mode_action.cpp)
ament_target_dependencies(set_offboard_mode_action 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
  utilities
  rclcpp_action
)
install(TARGETS set_offboard_mode_action DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
