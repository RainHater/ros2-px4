#arm状态
uint8 ARM_DISABLED              = 1             #arm上锁
uint8 ARM_ENABLE                = 2             #arm解锁

#offboard 模式
uint16 POSITION                 = 0x01
uint16 VELOCITY                 = 0x02
uint16 ACCELERATION             = 0x04
uint16 ATTITUDE                 = 0x08
uint16 BODY_RATE                = 0x10
uint16 THRUST_AND_TORQUE        = 0x20
uint16 DIRECT_ACTUATOR          = 0x40
uint16 OFFBOARD_DISABLED        = 0x80

uint8 arm                                   #arm 状态
uint16 offboard                             #offboard 状态
uint64 timestamp			                #time since system start (microseconds)
