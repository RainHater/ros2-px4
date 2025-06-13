cmake_minimum_required(VERSION 3.8)
project(utilities)

if(NOT CMAKE_CXX_STANDARD) 
  set(CMAKE_CXX_STANDARD 20)
endif() 
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# 1) 建立一个静态库或共享库
add_library(vehicle_command_helper
  src/vehicle_command_helper.cpp
)

# 2) 指定头文件搜索路径
target_include_directories(vehicle_command_helper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3) 添加库依赖（如果需要）
# target_link_libraries(vehicle_command_helper PUBLIC
#   rclcpp::rclcpp
#   px4_msgs::px4_msgs
# )

# 4) ament 导出依赖
ament_target_dependencies(vehicle_command_helper
  # 如果 link 了 rclcpp 或 px4_msgs，就放在这里：
  # rclcpp
  # px4_msgs
)

# 5) 安装规则：把库和头文件都安装到 ROS2 标准路径
install(
  TARGETS vehicle_command_helper
  EXPORT export_utilities
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 6) 导出 cmake config 给其他包 find_package 调用
ament_export_targets(export_utilities HAS_LIBRARY_TARGET)
ament_export_dependencies(
  # rclcpp
  # px4_msgs
)

ament_package()
