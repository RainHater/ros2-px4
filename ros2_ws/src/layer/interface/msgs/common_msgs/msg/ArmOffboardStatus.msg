uint8 OFFBOARD_NOT_ACTIVE   = 0     #offboard 未解锁
uint8 POSITION = 1                  #位置控制
uint8 VELOCITY = 2                  #速度控制
uint8 ATTITUDE = 3                  #姿态控制
uint8 LAND = 4                      #降落控制模式，原地垂直降落

uint8 ARMING_STATE_DISARMED = 1     #arm上锁
uint8 ARMING_STATE_ARMED    = 2     #arm解锁

uint8 offboard_mode                 #offboard模型
uint8 arming_state                  #arm解锁模式
