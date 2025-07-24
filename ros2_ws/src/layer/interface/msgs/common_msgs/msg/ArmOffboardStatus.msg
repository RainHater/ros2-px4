#arm状态
uint8 ARM_DISABLED          = 1             #arm上锁
uint8 ARM_ENABLE            = 2             #arm解锁

#offboard 模式
uint8 POSITION              = 0x01
uint8 VELOCITY              = 0x02
uint8 ACCELERATION          = 0x04
uint8 ATTITUDE              = 0x08
uint8 BODY_RATE             = 0x10
uint8 THRUST_AND_TORQUE     = 0x20
uint8 DIRECT_ACTUATOR       = 0x40
uint8 OFFBOARD_DISABLED     = 0x80
         
uint8 arm                                   #arm 状态
uint8 offboard                              #offboard 状态
