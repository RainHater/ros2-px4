uint8 TASK_UNKNOWN = 0          #状态未知, 未初始化, 或没有任务
uint8 TASK_IN_PROGRESS = 1      #当前任务正在执行中
uint8 TASK_COMPLETED = 2        #当前任务已成功完成，例如“已飞到目标位置”
uint8 TASK_FAILED = 3           #当前任务执行失败，例如“无法到达目标”或“飞控异常”

uint8 status                    #表示当前任务状态，值为上面四个之一
