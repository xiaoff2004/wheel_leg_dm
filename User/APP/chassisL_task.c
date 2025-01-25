/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
	*						 左下角的DM4310发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "chassisL_task.h"
#include "fdcan.h"
#include "VMC_calc.h"

#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "config.h"
#include "remote.h"
#include "dt7_task.h"

vmc_leg_t left;

float LQR_K_L[12] =
{-20.9282f, -3.7083f, -4.9246f, -6.5792f, 11.0064f, 1.8198f, 29.6233f, 4.0909f, 2.7354f, 3.7450f, 25.5208f, 0.7842f}
;
extern float Poly_Coefficient[12][4];


float jump_time2;
extern float jump_time;

PidTypeDef LegL_Pid;

uint32_t CHASSL_TIME = 1;

void ChassisL_task(void)
{
    while (INS . ins_flag == 0) {//等待加速度收敛
        osDelay(1);
    }
    ChassisL_init(&chassis_move, &left, &LegL_Pid);//初始化左边两个关节电机和左边轮毂电机的id和控制模式、初始化腿部

    while (1) {
        chassisL_feedback_update(&chassis_move, &left, &INS);//更新数据

        chassisL_control_loop(&chassis_move, &left, &INS, LQR_K_L, &LegL_Pid);//控制计算


        if (chassis_move . start_flag == 1) {
//            mit_ctrl(&hfdcan1, 0x05, 0.0f, 0.0f, 0.0f, 0.0f, left . torque_set[1]);//left.torque_set[1]
//            osDelay(CHASSL_TIME);
//            mit_ctrl(&hfdcan1, 0x07, 0.0f, 0.0f, 0.0f, 0.0f, left . torque_set[0]);
            osDelay(CHASSL_TIME);


            osDelay(CHASSL_TIME);
        } else if (chassis_move . start_flag == 0) {
            mit_ctrl(&hfdcan1, 0x07, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);//left.torque_set[1]
            osDelay(CHASSL_TIME);
            mit_ctrl(&hfdcan1, 0x05, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSL_TIME);
        }
    }
}

void ChassisL_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legl)
{
    const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};
    chassis_move.leg_set = LEGLEN_MIN;
    joint_motor_init(&chassis -> joint_motor[2], 7, MIT_MODE);//发送id为7
    joint_motor_init(&chassis -> joint_motor[3], 5, MIT_MODE);//发送id为5

    VMC_init(vmc);//给杆长赋值

    PID_init(legl, PID_POSITION, legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

    for (int j = 0; j < 10; j++) {
        enable_motor_mode(&hfdcan1, chassis -> joint_motor[2] . para . id, chassis -> joint_motor[3] . mode);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++) {
        enable_motor_mode(&hfdcan1, chassis -> joint_motor[3] . para . id, chassis -> joint_motor[2] . mode);
        osDelay(1);
    }
//    for (int j = 0; j < 10; j++) {
//        enable_motor_mode(&hfdcan2, chassis -> wheel_motor[1] . para . id, chassis -> wheel_motor[1] . mode);//左边轮毂电机
//        osDelay(1);
//    }
}

void chassisL_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins)
{
    vmc -> phi1 = pi / 2.0f + chassis -> joint_motor[2] . para . pos + 0.10472f;//因无法准确设置零点，故加上一个偏置
    vmc -> phi4 = pi / 2.0f + chassis -> joint_motor[3] . para . pos - 0.10472f;

    chassis -> myPithL = 0.0f - ins -> Pitch;//估计是左边的pitch角度
    chassis -> myPithGyroL = 0.0f - ins -> Gyro[1];//估计是左边的pitch角速度

}


extern uint8_t right_off_ground_flag;
uint8_t left_off_ground_flag;
extern float jump_time;

void chassisL_control_loop(chassis_t *chassis, vmc_leg_t *vmcl, INS_t *ins, float *LQR_K, PidTypeDef *leg)
{
    static float data_err[6]={0};
    VMC_calc_1_left(vmcl, ins,
                    ((float) CHASSL_TIME) * 3.0f / 1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒

    for (int i = 0; i < 12; i++) {
//        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcl -> L0);
        LQR_K[i] = LQR_K_L[i];
    }

    data_err[0] = 0.0f
                    - vmcl -> theta
		;
    data_err[1] = 0.0f
                    - vmcl -> d_theta
		;
    data_err[2] = 0.0f
           - chassis -> x_set + chassis -> x_filter
		;
    data_err[3] = 0.0f
            -chassis -> v_set +chassis -> v_filter
		;
    data_err[4] = 0.0f
            - chassis -> myPithL
            ;
    data_err[5] = 0.0f
            - chassis -> myPithGyroL
            ;


    chassis -> wheel_motor[1] . wheel_T = (LQR_K[0] * data_err[0]
                                           + LQR_K[1] * data_err[1]
                                           + LQR_K[2] * data_err[2]
                                           + LQR_K[3] * data_err[3]
                                           + LQR_K[4] * data_err[4]
                                           + LQR_K[5] * data_err[5]);
    chassis -> wheel_motor[1] . wheel_T *= 1.0f;
//    chassis -> wheel_motor[1] . wheel_T *= 0.0f;


    //右边髋关节输出力矩
    vmcl -> Tp = (LQR_K[6] * data_err[0]
                  + LQR_K[7] * data_err[1]
                  + LQR_K[8] * data_err[2]
                  + LQR_K[9] *  data_err[3]
                  + LQR_K[10] * data_err[4]
                  + LQR_K[11] * data_err[5]);
    vmcl -> Tp *=-1.0f;
//    vmcl -> Tp *=0.0f;

//    vmcl -> Tp = vmcl -> Tp + chassis -> anti_split_tp;//髋关节输出力矩
//
//    chassis -> wheel_motor[1] . wheel_T = chassis -> wheel_motor[1] . wheel_T - chassis -> turn_T;    //轮毂电机输出力矩

    mySaturate(&chassis -> wheel_motor[1] . wheel_T , -1.0f * DJI3508_TOQUE_MAX, DJI3508_TOQUE_MAX);//输出限幅

//    if(chassis -> jump_flag2 == 1 || chassis -> jump_flag2 == 2 || chassis -> jump_flag2 == 3)
//    {
//        if(chassis -> jump_flag2 == 1)
//        {//压缩阶段
//            vmcl -> F0 = MG / arm_cos_f32(vmcl -> theta) + PID_Calc(leg, vmcl -> L0, 0.08f);//前馈+pd
//
//            if(vmcl -> L0 < 0.10f)
//            {
//                jump_time2 ++;
//            }
//            if(jump_time2 >= 10 && jump_time >= 10)
//            {
//                jump_time2 = 0;
//                jump_time = 0;
//                chassis -> jump_flag2 = 2;
//                chassis -> jump_flag = 2;//压缩完毕进入上升加速阶段
//            }
//        }
//        else if(chassis -> jump_flag2 == 2)
//        {//上升加速阶段
//            vmcl -> F0 = MG / arm_cos_f32(vmcl -> theta) + PID_Calc(leg, vmcl -> L0, 0.40f);//前馈+pd
//
//            if(vmcl -> L0 > 0.18f)
//            {
//                jump_time2 ++;
//            }
//            if(jump_time2 >= 2 && jump_time >= 2)
//            {
//                jump_time2 = 0;
//                jump_time = 0;
//                chassis -> jump_flag2 = 3;
//                chassis -> jump_flag = 3;//上升完毕进入缩腿阶段
//            }
//        }
//        else if(chassis -> jump_flag2 == 3)
//        {//缩腿阶段
//            vmcl -> F0 = PID_Calc(leg, vmcl -> L0, 0.10f);//pd
//            chassis -> theta_set = 0.0f;
//            if(vmcl -> L0 < 0.15f)
//            {
//                jump_time2 ++;
//            }
//            if(jump_time2 >= 3 && jump_time >= 3)
//            {
//                jump_time2 = 0;
//                jump_time = 0;
//                chassis -> leg_set = 0.10f;
//                chassis -> last_leg_set = 0.10f;
//                chassis -> jump_flag2 = 0;
//                chassis -> jump_flag = 0;
//            }
//        }
//    }
//    else
//    {
    vmcl -> F0 =
            MG / arm_cos_f32(vmcl -> theta)
            + PID_Calc(leg, vmcl -> L0, chassis -> leg_set)
            ;//前馈+pd

//    }

//    left_off_ground_flag = ground_detectionL(vmcl, ins);//离地检测
    left_off_ground_flag =0;
//    if (chassis -> recover_flag == 0) {//倒地自起不需要检测是否离地
//        if ((right_off_ground_flag == 1 && left_off_ground_flag == 1 && vmcl -> leg_flag == 0 &&
//             chassis -> jump_flag2 != 1 &&
//             chassis -> jump_flag != 1 && chassis -> jump_flag2 != 2 && chassis -> jump_flag != 2)
//            || chassis -> jump_flag2 == 3) {//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
//            //排除跳跃的压缩阶段和跳跃的缩腿阶段
//            chassis -> wheel_motor[1] . wheel_T = 0.0f;
//            vmcl -> Tp = LQR_K[6] * (vmcl -> theta - 0.0f) + LQR_K[7] * (vmcl -> d_theta - 0.0f);
//
//            chassis -> x_filter = 0.0f;
//            chassis -> x_set = chassis -> x_filter;
//
//            vmcl -> Tp = vmcl -> Tp + chassis -> anti_split_tp;
//        } else {//没有离地
            vmcl -> leg_flag = 0;//置为0

//            if (chassis -> jump_flag2 == 0) {//不跳跃的时候需要roll轴补偿

//                vmcl -> F0 = vmcl -> F0 - chassis -> roll_f0;//roll轴补偿取反然后加上去

//            }
//        }
//    } else if (chassis -> recover_flag == 1) {
//        vmcl -> Tp = 0.0f;
//        vmcl -> F0 = 0.0f;
//    }

    mySaturate(&vmcl -> F0, -MAX_F0, MAX_F0);//限幅

    VMC_calc_2(vmcl);//计算期望的关节输出力矩

    if (chassis -> jump_flag2 == 1 || chassis -> jump_flag2 == 2 || chassis -> jump_flag2 == 3) {//跳跃的时候需要更大扭矩
        mySaturate(&vmcl -> torque_set[1], -1.0f * DM4310_TOQUE_MAX, DM4310_TOQUE_MAX);
        mySaturate(&vmcl -> torque_set[0], -1.0f * DM4310_TOQUE_MAX, DM4310_TOQUE_MAX);
    } else {//不跳跃的时候最大为额定扭矩
        mySaturate(&vmcl -> torque_set[1], -1.0f * DM4310_TOQUE_MAX, DM4310_TOQUE_MAX);
        mySaturate(&vmcl -> torque_set[0], -1.0f * DM4310_TOQUE_MAX, DM4310_TOQUE_MAX);
    }

}


