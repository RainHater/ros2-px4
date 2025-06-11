# PX4 ROS 2 Drone Project — MicoAir743v2 + RK3566

本项目是基于 PX4 和 ROS 2 的无人机平台，支持 RK3566 边缘计算平台与 MicoAir743v2 飞控，集成了自定义 ROS 2 功能包、SDF 仿真模型与硬件设计文件。

## 🚀 项目结构概览

```bash
rk3566-px4/
├── docs/           # 项目文档与资料
├── hardware/       # 电路板设计与外壳结构
├── px4/            # PX4 固件与自定义模块
├── ros2_ws/        # ROS 2 工作区
├── scripts/        # 常用脚本
├── simulation/     # 仿真模型与世界
```


## 🧠 系统架构

- 板载计算机：RK3566（Cortex-A55，运行 Ubuntu + ROS 2）
- 飞控单元：MicoAir743v2（PX4 固件）
- 通信方式：MAVLink / uORB / DDS（FastRTPS）
- 支持仿真：Gazebo Classic + SDF 模型

## 📦 硬件平台说明

| 模块         | 描述                                       |
|--------------|--------------------------------------------|
| 飞控         | MicoAir743v2，基于 STM32H7，支持 PX4       |
| 计算平台     | RK3566，运行 Ubuntu 22.04 和 ROS 2 Humble  |
| PCB 设计     | 位于 `hardware/pcb/`，包含原理图和布局文件 |
| 外壳设计     | 位于 `hardware/enclosure/`，包含 STEP/STL  |
| 传感器接口   | 支持 UART / SPI / CAN / I2C                |

## 📂 子模块说明

### PX4 固件（`px4/`）

- `PX4-Autopilot/`：PX4 源码（子模块或 fork）
- `custom_modules/`：自定义 PX4 模块（如任务扩展、控制算法）
- `cmake-configs/`：编译配置、板卡定义等

### ROS 2 工作区（`ros2_ws/`）

- `my_robot_nodes/`：功能节点（如图像处理、路径规划）
- `my_robot_description/`：机器人模型（URDF / SDF）
- `my_robot_bringup/`：启动文件集合
- `my_robot_interfaces/`：ROS 自定义接口（`msg/`, `srv/`）

### simulation/

- `sdf_models/`：Gazebo SDF 模型
- `launch/`：仿真启动文件
- `world/`：自定义仿真环境世界文件

## 🛠️ 开发与部署指南

请参考文档：[docs/setup_guide.md](docs/setup_guide.md)

主要步骤：

1. 克隆仓库并初始化子模块；
2. 构建 PX4 固件；
3. 构建 ROS 2 工作区；
4. 启动仿真环境或部署到 RK3566 实机；
5. 使用 `scripts/launch_all.sh` 启动系统。

## 🧩 文件与规范说明

- `hardware/`：原理图（`.sch`）、布局图（`.pcb`）、Gerber（`outputs/`）、3D 结构（`.step`, `.stl`）；
- `docs/`：datasheets、设计笔记、环境配置说明；
- `images/`：照片、渲染图、结果图；
- `.gitignore`：已忽略中间编译目录。

## 📄 License

本项目遵循 [MIT License](LICENSE)。


## 🛠️ 快速开始

### 环境准备

1. 安装 ROS 2 Humble（Ubuntu 22.04）
2. 安装 PX4 构建依赖（参考 PX4 官方文档）
3. 安装 Gazebo Classic 模拟器

### 项目初始化

```bash
# 克隆主项目并初始化子模块
git clone https://github.com/yourname/rk3566-px4.git
cd rk3566-px4
git submodule update --init --recursive