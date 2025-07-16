## 工作区结构
```bash
.
├── layer
│   ├── application [应用层]
│   │   ├── debug [调试数据包，比如查看pid波形]
│   │   ├── mission_planner [任务规划]
│   │   └── real_machine_test [实机测试任务]
│   ├── control [控制层]
│   │   ├── attitude_controller [（未使用）：姿态控制包]
│   │   ├── control_interface [对外暴露控制命令的接口，上层直接调用]
│   │   ├── motion_controller [运动控制包]
│   │   └── task_executor [一些控制任务逻辑，封装一些动作]
│   ├── interface [接口层]
│   │   ├── flight_controller_interface [和飞控固件通信，订阅/发布飞控状态、发送控制指令]
│   │   ├── msgs [官方消息的包定义]
│   │   └── px4_ros_com [px4官方库]
│   └── perception [感知层]
│       ├── state_estimator [状态估计（位置、姿态、速度融合）]
│       └── vision_pipeline [视觉识别算法]
├── simulation [仿真文件]
│
└── utilities [工具包]
```

> _日期：2025-07-07_  
> _作者：xuguocai_  
