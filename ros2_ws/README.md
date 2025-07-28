## 工作区结构
```bash
./src/
├── layer
│   ├── application [应用层]
│   │   └── mission_planner [任务规划]
│   │
│   ├── control [控制层]
│   │   ├── control_interface   [控制接口包]
│   │   │
│   │   └── control_motion  [控制包]
│   │
│   ├── interface   [接口层]
│   │   ├── flight_controller_interface [飞行控制器接口]
│   │   │
│   │   ├── msgs    [消息定义]
│   │   │   ├── common_msgs [自定义消息]
│   │   │   └── px4_msgs    [官方msgs包]
│   │   │
│   │   └── px4_ros_com [官方库]
│   │
│   └── perception  [感知层]
├── simulation  [仿真包]
│
└── utilities   [工具包]
```

> _日期：2025-07-28_  
> _作者：xuguocai_  
