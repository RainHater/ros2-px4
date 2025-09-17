## 工作区结构
```bash
.
├── scripts             [存放一些脚本]
│   ├── com             [存放公共脚本]
│   ├── sim             [存放仿真文件脚本]
│   ├── sync            [存放同步代码脚本]
│   └── target          [存放板载计算机运行的脚本]
├── src
│   ├── layer
│   │   ├── application [应用层]
│   │   ├── control     [控制层]
│   │   ├── interface   [接口层]
│   │   └── perception  [感知层]
│   ├── simulation      [仿真包]
│   └── utilities       [工具包]
│
└── tools               [工具可进行日志数据处理]
```

## 运行跟踪节点
### 仿真
```bash
ros2 launch simulation visual_track.launch.py
```

### 实物
```bash
ros2 launch mission_planner visual_track.launch.py
```

可在yaml文件里修改运行的参数来实现动态调参

## 一些脚本的使用

### 同步代码
```bash
#主机直接同步到板载计算机
bash scripts/sync/host_arm.sh

#经过arm设备编译发送
bash scripts/sync/host.sh
#在arm设备编译后
bash scripts/sync/ros2_ws.sh
```

### 仿真文件的复制
```bash
bash scripts/sim/copy_models_to_px4.sh
```

> _日期：2025-09-17_  
> _作者：xuguocai_  
