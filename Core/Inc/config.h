#ifndef __CONFIG_H__
#define __CONFIG_H__

#define LEGLEN_MIN (0.25f)  //原始腿长
#define LEGLEN_MAX (0.4f)  //最大腿长
#define INIT_LEGLEN (0.30f)  //初始腿长

#define INIT_PITH (0.00f)  //初始俯仰角

#define ROLL_MAX_ABS (0.40f)  //最大横滚角

//#define MG (79.5f)
#define MG (10.0f)
#define REDUCTION_RATIO (268.0f / 17.0f) //减速比
#define WHEEL_RADIUS (0.129f) //轮子直径

#define DJI3508_TOQUE_CONSTANT  (0.26f)//电机转矩常数
#define DJI3508_TOQUE_MAX  (2.46f)//电机最大转矩//3.0f *268.0f/17.0f/3591.0f*187.0f    2.46f

#define DM4310_TOQUE_MAX (20.0f)

#define MAX_F0 (100.0f)

#define LEG_PID_KP  80.0f
#define LEG_PID_KI  0.0f//不积分
#define LEG_PID_KD  5.0f
#define LEG_PID_MAX_OUT  100.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 140.0f
#define ROLL_PID_KI 0.0f
#define ROLL_PID_KD 10.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 30.0f
#define TP_PID_KI 0.0f
#define TP_PID_KD 1.0f
#define TP_PID_MAX_OUT  3.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.5f
#define TURN_PID_KI 0.0f
#define TURN_PID_KD 0.3f
#define TURN_PID_MAX_OUT  1.0f
#define TURN_PID_MAX_IOUT 0.0f



#define SENSE_VX  (-0.004f)
#define SENSE_YAW (-0.0002f)
#define SENSE_ROLL  (-0.00003f)
#define SENSE_LEGLEN (-0.000008f)

//2.23 1.57 0.91
//2.21      0.88

#endif
