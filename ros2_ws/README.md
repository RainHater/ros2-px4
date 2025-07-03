# 本系统基于 PX4 飞控与 ROS 2 Humble 框架，分为以下四层，以包为层级：

- **接口层(interface_layer)**：负责对外通信、协议转换、消息封装和解包，是上下游数据的“桥梁”和“翻译官”，包括和飞控固件、外设、外部系统的接口，以及上层和感知、控制层之间的消息传递接口。
- **感知层(perception_layer)**：处理传感器数据，完成环境感知、状态估计、目标检测和识别，负责数据融合、滤波、地图构建等感知任务。
- **应用层(application_layer)**：实现高层功能逻辑，如任务规划、人机交互、自动飞行策略等，是整个系统的“大脑”，调用感知和控制层完成复杂动作。
- **控制层(control_layer)**：根据感知层的状态和目标，计算控制命令，执行闭环控制包括飞行姿态控制、速度控制、任务执行等。
---

## **层级的包划分**

### **接口层（interface_layer）**
- flight_controller_interface：和飞控固件通信，订阅/发布飞控状态、发送控制指令
- communication_interface（未创建）：和外部系统（地面站、云端）的通信协议处理（比如MAVLink、ROS Topic）

---


### **感知层（perception_layer）**
- state_estimator：状态估计（位置、姿态、速度融合）
- vision_pipeline：视觉识别算法、

---

### **应用层（application_layer）**
- mission_planner：任务规划
- debug：调试数据包，比如查看pid波形

---

### **控制层（control_layer）**
- attitude_controller（未使用）：姿态控制包
- control_interface：对外暴露控制命令的接口，上层直接调用
- motion_controller：运动控制包
- task_executor：一些控制任务逻辑，封装一些动作

---
