# 设计说明文档（Design Notes）

本文件用于记录 PX4 + ROS 2 无人机平台的系统设计方案、模块分工、技术选型、接口规范和开发注意事项，便于团队成员理解项目架构与协作。

## 1. 项目目标

构建一个基于 RK3566 计算平台与 MicoAir743v2 飞控的模块化无人机系统，支持：

- PX4 飞控功能（姿态控制、飞行模式管理等）
- ROS 2 功能扩展（图像识别、路径规划、地面站通信）
- 仿真验证（Gazebo Classic + SDF）
- 板载自主飞行任务部署

---

## 2. 系统总体架构

```lua
               +----------------------+
               |      上位机 (PC)     |
               | Gazebo / RViz / ROS  |
               +----------+-----------+
                          |
                          | DDS / ROS 2
                          |
               +----------v-----------+
               |   RK3566 (ROS 2)     |
               |  my_robot_nodes/     |
               |  图像识别 / 控制逻辑 |
               +----------+-----------+
                          | MAVLink (串口)
                          |
               +----------v-----------+
               |  MicoAir743v2 (PX4)  |
               | 姿态控制 / 飞行控制  |
               +----------------------+

```


---

## 3. 飞控模块设计（PX4）

路径：`px4/`

### 自定义模块（custom_modules/）

- `offboard_control.cpp`：板载自主任务接口（订阅 ROS2 topic，控制无人机轨迹）
- `object_follow.cpp`：目标跟踪控制器，结合图像识别结果
- `px4_gps_bridge.cpp`：板载 GPS 数据重定向至 ROS 2

### 配置文件（cmake-configs/）

- `boards/micoair743v2/`：针对 MicoAir743v2 的 board 配置
- `cmake/configs/`：自定义构建模板

---

## 4. ROS 2 模块设计（ros2_ws/src/）

### `my_robot_nodes/`

- `object_tracker_node.cpp`：订阅摄像头图像，推理目标位置，发布 `/target_pose`
- `offboard_commander_node.cpp`：接收导航目标，生成轨迹，转为 MAVROS 命令
- `telemetry_logger.cpp`：订阅 `/vehicle_odometry` 和 `/sensor_combined`，记录飞行数据

### `my_robot_interfaces/`

- `msg/TargetPose.msg`
- `srv/SetMissionMode.srv`

### `my_robot_bringup/`

- 启动多个 ROS 2 节点的组合文件，支持模拟或真实飞控启动流程

---

## 5. 仿真系统设计（simulation/）

- `sdf_models/`：包含 RK3566 + 飞控 + 多传感器模型（URDF/SDF）
- `launch/simulation.launch.py`：自动启动 Gazebo + ROS 2 节点
- `world/`：自定义仿真环境（起飞平台 / 室外场景）

---

## 6. 数据流说明

### 图像目标跟踪路径

1. `V4L2 摄像头` → `/camera/image_raw`
2. `object_tracker_node` → `/target_pose`
3. `offboard_commander_node` → `/trajectory_setpoint`
4. MAVLink → PX4 `uORB` → 姿态控制器

---

## 7. 硬件设计（hardware/）

- `pcb/`：主板使用 STM32 外围扩展 RK3566 传感器数据
- `enclosure/`：便于散热与防震设计
- `BOM/`：含主要器件选型说明，匹配 PX4 推荐器件库

---

## 8. 通信协议说明

- PX4 ←→ ROS 2：通过 MAVLink 串口（可扩展为 microDDS）
- ROS 2 节点间：基于 DDS（FastRTPS）
- 外部设备（如地面站）：通过 MAVROS、UART 或 UDP

---

## 9. 开发约定

- ROS 2 使用 Humble 版本，节点支持参数化启动
- 所有自定义消息需在 `my_robot_interfaces` 中声明
- ROS 与 PX4 的 ID 配置保持一致
- 所有路径使用 `ament_index_python` 获取，避免硬编码

---

## 10. 未来扩展计划

- 增加板载视觉 SLAM 模块（如 ORB-SLAM3）
- 增加多机通信协议支持（Swarm 编队）
- 接入边缘 AI 模块（如 RKNN 推理加速）

---

## 附录

- PX4 固件版本：v1.14+
- ROS 2 版本：Humble Hawksbill
- 飞控型号：MicoAir743v2（STM32H7 系列）
- 板载平台：Firefly RK3566 Ubuntu 22.04
