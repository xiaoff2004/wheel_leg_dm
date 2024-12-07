#ifndef ALGORITHM_H
#define ALGORITHM_H
#include "stdint.h"

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))



void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);
void vofa_demo(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8);

#endif // DEBUG
