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

### 全部话题
```bash
/fmu/in/actuator_motors
/fmu/in/actuator_servos
/fmu/in/arming_check_reply
/fmu/in/aux_global_position
/fmu/in/config_control_setpoints
/fmu/in/config_overrides_request
/fmu/in/distance_sensor
/fmu/in/goto_setpoint
/fmu/in/manual_control_input
/fmu/in/message_format_request
/fmu/in/mode_completed
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
/fmu/in/register_ext_component_request
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/trajectory_setpoint
/fmu/in/unregister_ext_component
/fmu/in/vehicle_attitude_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_command_mode_executor
/fmu/in/vehicle_mocap_odometry
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/vehicle_visual_odometry
/fmu/out/airspeed_validated
/fmu/out/arming_check_request
/fmu/out/battery_status
/fmu/out/collision_constraints
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/home_position
/fmu/out/manual_control_setpoint
/fmu/out/message_format_response
/fmu/out/mode_completed
/fmu/out/position_setpoint_triplet
/fmu/out/register_ext_component_reply
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_command_ack
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_land_detected
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status_v1
/fmu/out/vtol_vehicle_status
/parameter_events
/rosout
```

> _日期：2025-09-17_  
> _作者：xuguocai_  
