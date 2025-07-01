cmake_minimum_required(VERSION 3.8)
project(application)

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
find_package(utilities REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control REQUIRED)

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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

add_executable(mission_planner_node src/mission_planner_node.cpp)
ament_target_dependencies(mission_planner_node 
  rclcpp 
  px4_msgs 
  geometry_msgs
  common_msgs
  rclcpp_action
  control
  utilities
)

install(TARGETS mission_planner_node DESTINATION lib/${PROJECT_NAME})

ament_package()
