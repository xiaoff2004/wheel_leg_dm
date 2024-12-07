/*
 * @file		remote.c/h
 * @brief		遥控器串口中断
 * @history
 * 版本			作者			编写日期				更新内容
 * v1.0.0		姚启杰		2023/4/1
 * v1.1.1		许金帅		2023/4/12			支持接收机热拔插
 * v1.1.2		姚启杰		2024/4/1			更改解算触发方式
 */
#include "remote.h"
#include "usart.h"
#include "main.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include "stm32h7xx_hal_dma.h"
#include "dma.h"

#define Remote_CHANNAL_ERROR_VALUE 700
#define REMOTE_RXBUFF_SIZE 54

RC_Ctl_t RC_Ctl;
uint8_t remote_rxbuff[REMOTE_RXBUFF_SIZE]={0};

uint8_t Remote_data_is_error(void);



void RemoteSolve()
{
	RC_Ctl.rc.ch1 = (remote_rxbuff[0] | remote_rxbuff[1] << 8) & 0x07FF;
	RC_Ctl.rc.ch1 -= 1024;
	RC_Ctl.rc.ch2 = (remote_rxbuff[1] >> 3 | remote_rxbuff[2] << 5) & 0x07FF;
	RC_Ctl.rc.ch2 -= 1024;
	RC_Ctl.rc.ch3 = (remote_rxbuff[2] >> 6 | remote_rxbuff[3] << 2 | remote_rxbuff[4] << 10) & 0x07FF;
	RC_Ctl.rc.ch3 -= 1024;
	RC_Ctl.rc.ch4 = (remote_rxbuff[4] >> 1 | remote_rxbuff[5] << 7) & 0x07FF;
	RC_Ctl.rc.ch4 -= 1024;
	/* prevent remote control zero deviation */
	if (RC_Ctl.rc.ch1 <= 5 && RC_Ctl.rc.ch1 >= -5)
	{
		RC_Ctl.rc.ch1 = 0;
	}
	if (RC_Ctl.rc.ch2 <= 5 && RC_Ctl.rc.ch2 >= -5)
	{
		RC_Ctl.rc.ch2 = 0;
	}
	if (RC_Ctl.rc.ch3 <= 5 && RC_Ctl.rc.ch3 >= -5)
	{
		RC_Ctl.rc.ch3 = 0;
	}
	if (RC_Ctl.rc.ch4 <= 5 && RC_Ctl.rc.ch4 >= -5)
	{
		RC_Ctl.rc.ch4 = 0;
	}
	RC_Ctl.rc.sw1 = ((remote_rxbuff[5] >> 4) & 0x000C) >> 2;
	RC_Ctl.rc.sw2 = (remote_rxbuff[5] >> 4) & 0x0003;
	RC_Ctl.rc.wheel = (remote_rxbuff[16] | remote_rxbuff[17] << 8) - 1024;
	if (Remote_data_is_error())
	{
	    memset(&RC_Ctl, 0, sizeof(RC_Ctl));
		RC_Ctl.rc.ch1 = 0;
    	RC_Ctl.rc.ch2 = 0;
    	RC_Ctl.rc.ch3 = 0;
    	RC_Ctl.rc.ch4 = 0;
    	RC_Ctl.rc.sw1 = RC_SW_DOWN;
    	RC_Ctl.rc.sw2 = RC_SW_DOWN;
		RemoteInit();
	    return ;
	}
}

//判断遥控器数据是否出错，
uint8_t Remote_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if ((RC_Ctl.rc.ch1) > Remote_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if ((RC_Ctl.rc.ch2) > Remote_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if ((RC_Ctl.rc.ch3) > Remote_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if ((RC_Ctl.rc.ch4) > Remote_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_Ctl.rc.sw1 == 0)
    {
        goto error;
    }
    if (RC_Ctl.rc.sw2 == 0)
    {
        goto error;
    }
    return 0;

    error:
    return 1;
}


HAL_StatusTypeDef state;

void RemoteInit(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_rxbuff, REMOTE_RXBUFF_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		RemoteSolve();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_rxbuff, REMOTE_RXBUFF_SIZE);
	}
}


