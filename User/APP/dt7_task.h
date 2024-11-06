#ifndef __DT7_TASK_H
#define __DT7_TASK_H

#include "main.h"
#include "chassisR_task.h"
#include "ins_task.h"

extern void dt7_task(void);
extern void dt7_data_process(RC_Ctl_t *RC_Ctl, chassis_t *chassis, float dt);

#endif

// start_flag = 1;使能标志
//recover_flag =1;恢复标志(恢复腿长，自启)
//prejump_flag = 1;预跳标志
//jump_flag = 1;jump_flag2 = 1;跳跃标志(左右腿)
