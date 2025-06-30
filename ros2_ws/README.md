# 本系统基于 PX4 飞控与 ROS 2 Humble 框架，分为以下四层，以包为层级：

- **接口层(interface)**：连接 PX4 飞控，桥接飞控与上层系统。
- **感知层(perception)**：通过当前飞控的自身数据来转换坐标。
- **控制层(control)**：实现飞控控制逻辑，如模式切换、轨迹控制。
- **应用层(application/vision_pipeline)**：完成任务决策、路径规划等高层功能。
---
