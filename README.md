# H743V2 + ROS 2 Humble + PX4 集成开发文档

## 环境与工具要求

- **操作系统**：Ubuntu 22.04 LTS
- **ROS2版本**：[Ros Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **PX4版本**：v1.15
- **飞控型号**：[H743V2_AIO（微空科技）](https://micoair.cn/docs/MicoAir743v2AIO45A-yi-ti-fei-kong-yong-hu-shou-ce)
- **飞控机架**：[MDV30（微空科技）](https://micoair.cn/docs/wei-kong-MVD30-3-cun-quan-quan-ji-jia)
- **通信中间件**：[Micro XRCE-DDS Agent](https://docs.px4.io/main/zh/middleware/uxrce_dds.html)
- **板载计算机**：[RDK X5](https://developer.d-robotics.cc/rdkx5)

## 安装与配置步骤

### Ros Humble 的安装
- 参考[Ros Humble 安装教程](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 项目的环境构建

1. 拉取源代码
```
git clone https://github.com/3519610554/ros2-px4.git --depth 1
```

2. 更新子模块仓库
```bash
cd ros2-px4/ && git submodule update --init --recursive --depth 1
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
sudo apt install ros-humble-cv-bridge ros-humble-tf2 ros-humble-tf-transformations ros-humble-vision-opencv ros-dev-tools -y
sudo apt install python3-colcon-common-extensions libyaml-cpp-dev -y
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
1. 创建服务文件
```bash
sudo nano /etc/systemd/system/microxrceagent.service
```

2. 把下面内容粘贴进去
```ini
[Unit]
Description=Micro XRCE Agent serial service
After=network.target

[Service]
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/sunrise/Desktop/ros2_px4/install/setup.bash && /usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyS7 -b 921600"
Restart=always
RestartSec=3
User=root

[Install]
WantedBy=multi-user.target
```

3. 刷新 systemd 配置并启用服务
```bash
sudo systemctl daemon-reload
sudo systemctl enable microxrceagent.service
```

4. 立即启动服务
```bash
sudo systemctl start microxrceagent.service
```

5. 验证是否运行成功
```bash
systemctl status microxrceagent.service
```

## 上传代码到板载计算机上
```bash
bash scripts/sync_ros2_ws.sh 192.168.0.44
```

## 连接板载计算机的数传
1. 创建一个 systemd 服务脚本：
```bash
sudo nano /etc/systemd/system/serial-port-setup.service
```

2. 填入以下内容
```ini
[Unit]
Description=Set serial port baud rate
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/stty -F /dev/ttyS1 57600

[Install]
WantedBy=multi-user.target
```

3. 启用该服务：
```bash
sudo systemctl daemon-reexec
sudo systemctl enable serial-port-setup.service
sudo systemctl start serial-port-setup.service
```

重启
```bash
sudo systemctl daemon-reload
sudo systemctl restart serial-getty@ttyS1.service
```

## **注意点:**
当 clangd 解析不到ros的代码时，可以安装 libstdc++-12-dev 来解决问题
```bash
sudo apt install --reinstall libstdc++-12-dev
```

> _日期：2025-08-08_  
> _作者：xuguocai_  
