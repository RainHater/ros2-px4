cmake_minimum_required(VERSION 3.8)
project(mission_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(utilities REQUIRED)
find_package(task_executor REQUIRED)

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

function(register_node target_name)
  add_executable(${target_name} src/${target_name}.cpp)
  ament_target_dependencies(${target_name}
    rclcpp
    rclcpp_action
    px4_msgs 
    geometry_msgs
    common_msgs
    utilities
  )
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
  target_link_libraries(${target_name} task_executor::api)
endfunction()

register_node(mission_planner_node)
register_node(simple_indoor_control_task)
register_node(vision_test_task)

ament_package()
