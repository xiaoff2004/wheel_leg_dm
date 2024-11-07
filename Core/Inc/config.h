#ifndef __CONFIG_H__
#define __CONFIG_H__

#define LEGLEN_MIN 0.2f  //原始腿长
#define LEGLEN_MAX 0.4f  //最大腿长


#define ROLL_MAX_ABS 0.40f  //最大横滚角

#define REDUCTION_RATIO ( 268.0f / 17.0f) //减速比
#define WHEEL_RADIUS 0.129f //轮子半径
#define DJI3508_TOQUE_CONSTANT  0.3f//电机转矩常数


#define SENSE_VX  (-0.004f)
#define SENSE_YAW (-0.0002f)
#define SENSE_ROLL  (-0.00003f)
#define SENSE_LEGLEN (-0.000008f)

#endif
