# PX4 ROS 2 Drone Project — MicoAir743v2 + 板载计算平台 (型号待定)

本项目基于 PX4 与 ROS 2 构建，支持 MicoAir743v2 飞控与待定型号的边缘计算平台，集成了以下内容：

- 自定义 ROS 2 功能包（视觉流水线、任务规划、飞控控制等）
- PX4 固件扩展模块
- 仿真模型与 Gazebo 支持
- 硬件设计文件

---

## 项目总体目录

```bash
rk3566-px4/         #项目根（名称可根据实际板载平台调整）
├── docs/           #项目文档与资料
├── hardware/       #电路板与外壳设计
│ ├── pcb/          #原理图（.sch）、布局（.pcb）、Gerber
│ └── enclosure/    #3D 外壳（.step/.stl）
├── px4/            #PX4 固件源码与自定义模块
├── ros2_px4_ws/    #ROS 2 工作区
├── simulation/     #仿真模型与世界文件
└── scripts/        #启动/部署等脚本
```

---

## 硬件平台说明

| 模块     | 描述                                   |
|----------|----------------------------------------|
| 飞控     | MicoAir743v2，基于 STM32H7，支持 PX4   |
| 计算平台 | 型号待定，运行 Ubuntu 22.04 + ROS 2 Humble |
| 传感器接口 | 支持 UART / SPI / CAN / I2C           |

---

## 软件子模块（ROS 2 工作区：`ros2_px4_ws/`）

```bash
ros2_px4_ws/ # ROS 2 工作区根
├── src/ # 所有功能包源码
│ ├── common_msgs/      # 自定义消息/服务/动作定义
│ ├── vision_pipeline/  # 视觉处理流水线
│ ├── flight_control/   # PX4 Offboard 控制
│ ├── mission_planner/  # 高层任务规划与编排
│ ├── rviz_configs/     # RViz 可视化配置
│ └── utilities/        # 通用工具与脚本
├── build/              # 构建产物
├── install/            # 安装结果
└── log/                # 构建/运行日志
```

### 各包职责

- **common_msgs**：集中定义项目内所有自研 msg/srv/action。
- **vision_pipeline**：相机驱动、图像预处理、目标检测与跟踪节点。
- **flight_control**：Arm/Disarm、Offboard 控制、状态监控节点。
- **mission_planner**：接收高层指令，管控视觉与飞控节点，实现任务流。
- **rviz_configs**：存放 rviz 配置及启动文件，支持可视化调试。
- **utilities**：参数加载、日志封装、启动脚本等公共代码。

---

## 开发与运行

1. 初始化：

```bash
git clone https://github.com/3519610554/ros2-px4.git
cd ros2-px4
git submodule update --init --recursive
```
2. 构建 PX4：进入 px4/ 目录，按照官方流程编译固件。

3. 构建 ROS 2：
```bash
cd ros2_px4_ws
colcon build --symlink-install
```


4. 运行：
- 仿真：
  ```
  ros2 launch simulation sim.launch.py
  ```
- 视觉流水线：
  ```
  ros2 launch vision_pipeline pipeline.launch.py
  ```
- 飞控控制：
  ```
  ros2 launch flight_control offboard.launch.py
  ```
- 任务管理：
  ```
  ros2 launch mission_planner mission.launch.py
  ```

---

## 仿真与扩展

- 仿真模型：simulation/sdf_models/ 与 simulation/world/。
- Gazebo Classic：支持 SITL + PX4 插件。
- 日志回放：集成 rosbag2。
- UI Dashboard：可搭建 rqt 或自定义前端。

---
