# Design Notes: 使用 DDS 实现 ROS 2 ↔ PX4 通信

## 1. 目标

通过 Data Distribution Service (DDS) 在 ROS 2 与 PX4 之间建立高性能、可配置的通信通道，替代传统的 RTPS/SITL-Bridge 方式，使多节点、多地面站环境下的数据交换更灵活可靠。

## 2. 通信需求

- **命令控制**  
  ROS 2 发布 OffboardControlMode、TrajectorySetpoint、VehicleCommand 到 PX4  
- **状态反馈**  
  PX4 发布 VehicleStatus、ActuatorOutput、SensorData 到 ROS 2  
- **带宽与实时**  
  对高频控制和状态数据应用低延迟、高可靠性 QoS  
- **多订阅者**  
  支持多地面站或监控节点同时订阅同一 PX4 话题

## 3. DDS 中间件选型

| 中间件      | 特性                  | 推荐理由                             |
|-------------|-----------------------|--------------------------------------|
| Fast DDS    | ROS 2 默认、生态成熟  | 与 ROS 2 Humble 集成，无需额外依赖    |
| Cyclone DDS | 资源占用低、嵌入式友好| 内存与线程可定制，适合资源受限板端    |
| RTI Connext | 企业级、工具链完善    | 提供专业监控、支持 DDS 安全扩展       |

**初期选型：Fast DDS**，保持与 ROS 2 默认中间件一致，简化部署与调试。

## 4. QoS 策略设计

- **OffboardControlMode / TrajectorySetpoint**  
  - Reliability：RELIABLE  
  - History：KEEP_LAST(1)  
  - Deadline：50 ms  
- **VehicleCommand (Arm/Disarm)**  
  - Reliability：RELIABLE  
  - History：KEEP_LAST(5)  
  - Lifespan：500 ms  
- **VehicleStatus / ActuatorOutput**  
  - Reliability：BEST_EFFORT  
  - History：KEEP_LAST(10)  
  - Deadline：100 ms  

## 5. 系统集成方案

1. **Flight Control 节点**  
   - 使用 rclcpp::QoS 构造发布器/订阅器  
   - 在 launch 文件中可选择不同中间件插件  
2. **PX4 端 uORB ↔ DDS**  
   - 在 Board（或边缘机）部署 Micro‑XRCE‑DDS Agent  
   - Agent 将 uORB 消息转换并发布到 DDS 网络  
   - 配置 XML 映射文件，定义 Topic 名称与 QoS  
3. **多机通信**  
   - 在同一 DDS Domain 下，ROS 2 节点与 Micro‑XRCE‑Agent 自动发现  

## 6. 话题映射示例

| ROS 2 话题                         | DDS Topic       | 消息类型                              |
|------------------------------------|-----------------|---------------------------------------|
| `/fmu/in/offboard_control_mode`    | `OffboardCMD`   | px4_msgs::msg::OffboardControlMode    |
| `/fmu/in/trajectory_setpoint`      | `TrajSetpoint`  | px4_msgs::msg::TrajectorySetpoint     |
| `/fmu/in/vehicle_command`          | `VehicleCmd`    | px4_msgs::msg::VehicleCommand         |
| `/fmu/out/vehicle_status`          | `VehicleStatus` | px4_msgs::msg::VehicleStatus          |
| `/fmu/out/actuator_outputs`        | `ActuatorOut`   | px4_msgs::msg::ActuatorMotors         |

## 7. 验证与测试

- **延迟测试**：使用 `ros2 topic hz` 对比 DDS 与传统桥接的端到端延迟  
- **丢包率评估**：在不同网络条件下统计消息丢失  
- **多订阅测试**：验证多地面站并发订阅性能与稳定性  

## 8. 后续扩展

- DDS 安全插件：身份认证、加密、访问控制  
- 跨域部署：不同子网／跨 WAN 场景  
- 可视化监控：Fast DDS Monitor 或 RTI Monitor  

> _日期：2025-06-13_  
> _作者：xuguocai_  
