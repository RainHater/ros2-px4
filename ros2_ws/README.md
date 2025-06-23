# 本系统基于 PX4 飞控与 ROS 2 Humble 框架，分为以下三层：

- **接口层**：连接 PX4 飞控，桥接飞控与上层系统（如 `px4_bridge_node`、`state_estimator_node`）；
- **控制层**：实现飞控控制逻辑，如模式切换、轨迹控制（如 `flight_mode_manager_node`、`offboard_ctrl_node`）；
- **应用层**：完成任务决策、路径规划等高层功能（如 `mission_planner_node`）。

---

## 系统节点分层与职责

| 层级       | 节点名                   | 功能说明                                               |
|------------|--------------------------|--------------------------------------------------------|
| 接口层     | `px4_bridge_node`        | 桥接 PX4 与 ROS，转发消息，PX4控制集中在这个节点处理        |
| 接口层     | `state_estimator_node`   | 接收 GPS，生成目标轨迹点，向控制层提供导航信息        |
| 控制层     | `flight_mode_manager_node`| 控制 Arm/Offboard 模式状态，管理 PX4 模式              |
| 控制层     | `offboard_ctrl_node`     | 控制 Offboard 模式运行，处理轨迹、位姿控制逻辑         |
| 应用层     | `mission_planner_node`   | 任务控制器，生成目标点，控制任务流程                   |

---

## 包划分

- **common_msgs**  
  自定义消息，自定义服务，自定义动作等。

- **flight_control**  
  负责与 PX4 通信、飞行模式管理、轨迹规划与执行、状态融合与安全监测。

- **vision_pipeline**  
  负责图像采集、预处理、目标检测/姿态估计、视觉伺服。

---
## topic 消息总览

### PX4 相关桥接节点(由 px4_bridge_node 提供)

| Topic                                     | 消息类型                                       | 方向         | 用途                                         |
|------------------------------------------|------------------------------------------------|--------------|----------------------------------------------|
| `/fmu/in/vehicle_command`                | `px4_msgs::msg::VehicleCommand`                | → PX4        | Arm/Disarm模式切换                         |
| `/fmu/in/trajectory_setpoint`            | `px4_msgs::msg::TrajectorySetpoint`            | → PX4        | 位置或速度 Setpoint                          |
| `/fmu/in/offboard_control_mode`          | `px4_msgs::msg::OffboardControlMode`           | → PX4        | 设置 Offboard 控制模式                      |
| `/fmu/out/vehicle_odometry`              | `px4_msgs::msg::VehicleOdometry`               | ← PX4        | PX4 内部融合后的姿态/位置/速度状态         |
| `/fmu/out/vehicle_global_position`       | `px4_msgs::msg::VehicleGlobalPosition`         | ← PX4        | PX4 提供的全球定位信息                      |
| `/fmu/out/vehicle_status_v1`             | `px4_msgs::msg::VehicleStatus`                 | ← PX4        | PX4 当前状态信息（是否解锁、飞行模式等）     |

### 与外部系统交互节点(由 px4_bridge_node 提供)

| Topic                                     | 消息类型                                       | 方向         | 用途                                         |
|------------------------------------------|------------------------------------------------|--------------|----------------------------------------------|
| `/interface/in/vehicle_command`          | `px4_msgs::msg::VehicleCommand`                | ← 控制  | 外部控制命令（桥接转发至 PX4）               |
| `/interface/in/trajectory_setpoint`      | `px4_msgs::msg::TrajectorySetpoint`            | ← 控制  | 外部轨迹点（桥接转发至 PX4）                 |
| `/interface/in/offboard_control_mode`    | `px4_msgs::msg::OffboardControlMode`           | ← 控制  | 外部 Offboard 模式请求                       |
| `/interface/out/vehicle_odometry`        | `px4_msgs::msg::VehicleOdometry`               | → 控制  | 桥接后的 PX4 融合位姿                        |
| `/interface/out/vehicle_global_position` | `px4_msgs::msg::VehicleGlobalPosition`         | → 控制  | 桥接后的 GPS 位置                            |
| `/interface/out/vehicle_status_v1`       | `px4_msgs::msg::VehicleStatus`                 | → 控制  | 桥接后的飞行器状态信息                      |

### flight_mode_manager_node节点: 管理offboard模式和arm解锁

| Topic                                      | 消息类型                                       | 方向          | 用途                                             |
|-------------------------------------------|------------------------------------------------|---------------|--------------------------------------------------|
| `/control/set_offboard_mode`              | `common_msgs::msg::ArmOffboardStatus`          | ← 控制     | 其他节点设置 PX4 offboard控制模式                |
| `/control/px4_mode_status_broadcaster`    | `common_msgs::msg::ArmOffboardStatus`          | → 控制     | 当前 PX4 控制模式（由 flight_mode_manager_node节点 发布）         |
| `/interface/in/vehicle_command`           | `px4_msgs::msg::VehicleCommand`                | → px4_bridge_node    | (转发PX4)                                |
| `/interface/in/offboard_control_mode`     | `px4_msgs::msg::OffboardControlMode`           | → px4_bridge_node    | 设置 offboard 控制模式(转发PX4)                          |
| `/interface/out/vehicle_status_v1`        | `px4_msgs::msg::VehicleStatus`                 | ← px4_bridge_node    | PX4 当前状态反馈(转发PX4)                                |

### mission_planner_node节点: 任务的规划

| Topic                                      | 消息类型                                       | 方向         | 用途                                            |
|-------------------------------------------|------------------------------------------------|--------------|-------------------------------------------------|
| `/control/target_gps`                     | `common_msgs::msg::TargetGps`                  | → state_estimator_node | 发布任务目标 GPS 点                         |
| `/control/px4_mode_status_broadcaster`    | `common_msgs::msg::ArmOffboardStatus`          | ← flight_mode_manager_node节点   | 获取当前 PX4 模式状态                          |

### offboard_ctrl_node节点: 根据offboard模式使用位置控制和速度控制

| Topic                                      | 消息类型                                       | 方向          | 用途                                            |
|-------------------------------------------|------------------------------------------------|---------------|-------------------------------------------------|
| `/interface/in/trajectory_setpoint`       | `px4_msgs::msg::TrajectorySetpoint`            | → px4_bridge_node    | 向 PX4 发布最终轨迹控制 setpoint               |
| `/interface/out/vehicle_odometry`         | `px4_msgs::msg::VehicleOdometry`               | ← px4_bridge_node    | 获取当前位置                                     |
| `/control/trajectory_setpoint`            | `common_msgs::msg::TrajectorySetPoint`         | ← state_estimator_node    | 获取轨迹规划结果                                 |
| `/control/px4_mode_status_broadcaster`    | `common_msgs::msg::ArmOffboardStatus`          | ← flight_mode_manager_node节点  | 获取当前飞控控制模式                             |

### state_estimator_node节点: 获取外部数据进行数据融合，现在暂时没做多少

| Topic                                      | 消息类型                                       | 方向          | 用途                                            |
|-------------------------------------------|------------------------------------------------|---------------|-------------------------------------------------|
| `/control/target_gps`                     | `common_msgs::msg::TargetGps`                  | ← mission_planner_node | 接收目标经纬度坐标 点                         |
| `/interface/out/vehicle_global_position`  | `px4_msgs::msg::VehicleGlobalPosition`         | ← px4_bridge_node     | 接收当前 gps 数据                               |
| `/control/trajectory_setpoint`            | `common_msgs::msg::TrajectorySetPoint`         | → 控制         | 发布规划后的位置 setpoint 给 Offboard 控制节点  |

---
