/*
 * @file		referee.c/h
 * @brief		遥控器串口中断
 * @history
 * 版本			作者			编写日期				更新内容
 * v1.0.0		姚启杰		2023/4/1
 * v1.1.1		许金帅		2023/4/12			支持接收机热拔插
 *
 */
#ifndef __REMOTE_H__
#define __REMOTE_H__
#include "main.h"

#define DR16_CH_VALUE_MIN ((uint16_t)364)
#define DR16_CH_VALUE_OFFSET ((uint16_t)1024)
#define DR16_CH_VALUE_MAX ((uint16_t)1684)
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

typedef struct
{
	struct
	{
	/* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
    int16_t wheel;
   	}rc;
 

}RC_Ctl_t;

extern RC_Ctl_t RC_Ctl;


extern void RemoteInit(void);


#endif
