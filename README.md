# H743V2 + ROS 2 Humble + PX4 集成开发文档

## 环境与工具要求

- **操作系统**：Ubuntu 22.04 LTS
- **ROS2版本**：[Ros Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **PX4版本**：v1.15
- **飞控型号**：[H743V2_AIO（微空科技）](https://micoair.cn/docs/MicoAir743v2AIO45A-yi-ti-fei-kong-yong-hu-shou-ce)
- **飞控机架**：[MDV30（微空科技）](https://micoair.cn/docs/wei-kong-MVD30-3-cun-quan-quan-ji-jia)
- **通信中间件**：[Micro XRCE-DDS Agent](https://docs.px4.io/main/zh/middleware/uxrce_dds.html)

## 安装与配置步骤

### Ros Humble 的安装
- 参考[Ros Humble 安装教程](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 项目的环境构建

1. 拉取源代码
```
git clone https://github.com/3519610554/ros2-px4.git
```

2. 更新子模块仓库
```bash
cd ros2-px4/ && git submodule update --init --recursive
```

3. 配置PX4的环境
```bash
bash ./px4/PX4-Autopilot/Tools/setup/ubuntu.sh
```

4. 安装DDS
```bash
cd px4/DDS/ && mkdir build
cd build
cmake ..
make -j16
sudo make install
```

5. 配置ros humble工具包
```bash
sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-vision-opencv ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
```

6. 配置python环境
- 返回到ros-px4主目录下
```bash
cd ros2_ws/
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
```

7. 构建编译:px4_msgs, px4_ros_com
```bash
colcon build --packages-select px4_msgs
source install/setup.bash
colcon build --packages-select px4_ros_com
```

---

## 板载计算机连接飞控
```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

## 上传代码到板载计算机上
```bash
sudo rsync -aAXv   --exclude="/.cache/"   --exclude="/.venv/"   --exclude="/build/"   --exclude="/install/"   --exclude="/log/"   --exclude="/tmp/"  --exclude="/src/simulation"   /home/ros2/Projects/ros2_px4/ros2_ws/   sunrise@192.168.0.160:/home/sunrise/Desktop/ros2_px4
```

> _日期：2025-07-07_  
> _作者：xuguocai_  
