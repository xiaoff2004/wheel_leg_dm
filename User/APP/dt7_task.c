/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     该任务是读取并处理ps2手柄传来的遥控数据，
    *            将遥控数据转化为期望的速度、期望的转角、期望的腿长等
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "dt7_task.h"
#include "cmsis_os.h"
#include "remote.h"
#include "config.h"
#include "vofa.h"




uint32_t DT7_TIME = 10; // ps2手柄任务周期是10ms

uint8_t last_flag = 0;
uint8_t flag = 0;
void dt7_task(void)
{
    RemoteInit();

    while (1)
    {                                           // 读数据
        dt7_data_process(&RC_Ctl, &chassis_move, (float)DT7_TIME / 1000.0f); // 处理数据，设置期望数据
        vofa_demo(
                left.L0,
                  right.L0,
                  0,
                  left.F0,
                  right.F0,
                   0,
                  0,
                  0);
        osDelay(DT7_TIME);
    }
}




extern vmc_leg_t right;
extern vmc_leg_t left;
void dt7_data_process(RC_Ctl_t *RC_Ctl, chassis_t *chassis, float dt)
{
    if (RC_Ctl->rc.sw1 != RC_SW_DOWN)
    {
        chassis->start_flag = 1;
        chassis->recover_flag = 0;

    }

    else
    {
        chassis->start_flag = 0;
        chassis->recover_flag = 0;
    }
//    if (chassis->recover_flag == 0
//    && ((chassis->myPithR < ((-3.1415926f) / 4.0f)
//    && chassis->myPithR > ((-3.1415926f) / 2.0f)) || (chassis->myPithR > (3.1415926f / 4.0f)
//    && chassis->myPithR < (3.1415926f / 2.0f))))
//    {
//        chassis->recover_flag = 1; // 需要自起
//        chassis->leg_set = LEGLEN_MIN;  // 原始腿长
//    }
//    if(RC_Ctl->rc.sw1 == RC_SW_UP  && chassis->prejump_flag == 0 && chassis->start_flag == 1)
//    {
//        chassis->prejump_flag = 1; // 预跳跃标志置1
//    }
//    else if (RC_Ctl->rc.sw1 != RC_SW_UP  && chassis->prejump_flag == 1)
//    {
//        chassis->prejump_flag = 0;
//    }
//
//    if (flag ==0  && RC_Ctl->rc.wheel>500 && chassis->prejump_flag == 1 && chassis->jump_flag == 0 && chassis->jump_flag2 == 0)
//    {
//        // 手柄上的左边的上下左右键的上按键被按下，开启跳跃
//        // 只有当预跳跃标志置1，按下这个键才能开启跳跃
//        chassis->jump_flag = 1;
//        chassis->jump_flag2 = 1;
//        flag = 1;
//    }
//
//    last_flag = flag;

//    if (chassis->start_flag == 1)
//    {                                                           // 启动
//        chassis->v_set = ((float)(RC_Ctl->rc.ch2)) * SENSE_VX; // 往前大于0
//        chassis->x_set = chassis->x_set + chassis->v_set * dt;
//
//        chassis->turn_set = chassis->turn_set + (RC_Ctl->rc.ch1) * SENSE_YAW; // 往右大于0
//
//        chassis->roll_set = chassis->roll_set + ((float)(RC_Ctl->rc.ch3)) * SENSE_ROLL;
//
//        mySaturate(&chassis->roll_set, -ROLL_MAX_ABS, ROLL_MAX_ABS);
//
//        chassis->leg_set = chassis->leg_set + ((float)(RC_Ctl->rc.ch4)) * SENSE_LEGLEN;
//        mySaturate(&chassis->leg_set, LEGLEN_MIN, LEGLEN_MAX);
//
//        if (fabsf(chassis->last_leg_set - chassis->leg_set) > 0.0001f)
//        {                       // 遥控器控制腿长在变化
//            right.leg_flag = 1; // 为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
//            left.leg_flag = 1;
//        }
//        chassis->last_leg_set = chassis->leg_set;
//    }
//    else if (chassis->start_flag == 0)
//    {                                           // 关闭
//        chassis->v_set = 0.0f;                  // 清零
//        chassis->x_set = chassis->x_filter;     // 保存
//        chassis->turn_set = chassis->total_yaw; // 保存
//        chassis->leg_set = LEGLEN_MIN;               // 原始腿长
//        chassis->roll_set = 0.0f;
//    }
}

