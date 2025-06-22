# ROS 2 Humble + PX4 飞控＋视觉一体化系统框架文档

## 目录

1. [包划分](#包划分)  
2. [消息总览](#消息总览)  
3. [节点详情](#节点详情)  
   - [flight_control](#flight_control_pkg)  
     - [px4_bridge_node](#px4_bridge_node)  
     - [flight_mode_manager_node](#flight_mode_manager_node)  
     - [offboard_control_node](#offboard_control_node)  
     - [state_estimator_node](#state_estimator_node)  
     - [mission_planner_node](#mission_planner_node)  
     - [safety_monitor_node](#safety_monitor_node)  
   - [vision_pipeline](#vision_pkg)  
     - [camera_driver_node](#camera_driver_node)  
     - [image_preprocessor_node](#image_preprocessor_node)  
     - [object_detection_node](#object_detection_node)  
     - [pose_estimation_node](#pose_estimation_node)  
     - [visual_servo_node](#visual_servo_node)  

---

## 包划分

- **flight_control_pkg**  
  负责与 PX4 通信、飞行模式管理、轨迹规划与执行、状态融合与安全监测。

- **vision_pkg**  
  负责图像采集、预处理、目标检测／姿态估计、视觉伺服。

---

## 消息总览

| Topic                            | 消息类型                                         | 方向       | 用途                                      |
|----------------------------------|-------------------------------------------------|-----------|-------------------------------------------|
| `/fmu/vehicle_command/in`        | `px4_msgs::msg::VehicleCommand`                 | → PX4     | Arm/Disarm、模式切换                      |
| `/fmu/trajectory_setpoint/in`    | `px4_msgs::msg::TrajectorySetpoint`             | → PX4     | 位置或速度 setpoint                       |
| `/robot_state`                   | `custom_msgs::msg::RobotState`                  | → 控制    | 融合后的位姿／速度／姿态状态              |
| `/mission/waypoints`             | `custom_msgs::msg::WaypointArray`               | → 控制    | 位置控制目标航点列表                      |
| `/mission/velocity_cmd`          | `geometry_msgs::msg::Twist`                     | → 控制    | 速度控制目标                              |
| `/flight_control/mode`           | `std_msgs::msg::UInt8`                          | → 控制    | 切换 POSITION/VELOCITY 模式               |
| `/mission/start_offboard`        | `std_msgs::msg::Bool`                           | → 模式管理| 触发 Arm + Offboard                       |
| `/mission/kill_offboard`         | `std_msgs::msg::Bool`                           | → 模式管理| 触发 Disarm                               |
| `/safety/emergency`              | `std_msgs::msg::Bool`                           | → 各节点  | 紧急 Disarm                                |
| `/vision/image_raw`              | `sensor_msgs::msg::Image`                       | → 视觉    | 原始图像                                  |
| `/vision/camera_info`            | `sensor_msgs::msg::CameraInfo`                  | → 视觉    | 相机内参                                  |
| `/vision/image_proc`             | `sensor_msgs::msg::Image`                       | → 视觉    | 预处理后图像                              |
| `/vision/detections`             | `vision_msgs::msg::Detection2DArray`            | → 视觉    | 目标检测结果                              |
| `/vision/pose`                   | `geometry_msgs::msg::PoseStamped`               | → 视觉&飞控 | 视觉估计的位姿                            |
| `/mission/goal`                  | `geometry_msgs::msg::PoseStamped`               | → 视觉&飞控 | 视觉伺服下发的新航点                      |

---

## 节点详情

### flight_control_pkg

#### px4_bridge_node
- **职责**：  
  - 与 PX4 通信：收发 MAVLink via px4_ros_com/px4_msgs  
  - 转发 `/fmu/vehicle_command/in` → PX4  
  - 转发 PX4 发布的 `/px4/state`、`/px4/odometry` → ROS

- **订阅**  
  - `/fmu/vehicle_command/in`  
- **发布**  
  - `/px4/state`  
  - `/px4/odometry`  

#### flight_mode_manager_node
- **职责**：  
  - 根据上层触发（`/mission/start_offboard`、`/mission/kill_offboard`、`/safety/emergency`）发布 `VehicleCommand` 完成 Arm/Offboard/Disarm

- **订阅**  
  - `/mission/start_offboard`  
  - `/mission/kill_offboard`  
  - `/safety/emergency`  
- **发布**  
  - `/fmu/vehicle_command/in`  

#### offboard_control_node
- **职责**：  
  - 根据当前模式（位置/速度）计算控制指令  
  - 发布 `TrajectorySetpoint` 到 `/fmu/trajectory_setpoint/in`

- **订阅**  
  - `/robot_state`  
  - `/mission/waypoints`  
  - `/mission/velocity_cmd`  
  - `/flight_control/mode`  
- **发布**  
  - `/fmu/trajectory_setpoint/in`  

- **控制模式枚举**  
  ```cpp
  #define POSITION_MODE 0
  #define VELOCITY_MODE 1
