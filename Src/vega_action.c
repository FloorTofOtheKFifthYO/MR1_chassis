/*******************************************************************************
Copyright:      2018/12/18
File name:      vega_action.c
Description:    action全场定位，使用前初始化十秒
Author:         徐铭远
Version：       1.0
Data:           2018/12/18 22:36
History:        无
Bug:            无
*******************************************************************************/
#include "vega_action.h"

void vega_action_reset()
{
	uprintf(ACTION_USART,"ACT0\n");
}
void vega_action_setAll(float pos_x, float pos_y, float angle)
{
	uprintf(ACTION_USART,"ACTA%f%f%f\n",angle,pos_x,pos_y);	
}
void vega_action_setPos(float pos_x, float pos_y)
{
	uprintf(ACTION_USART,"ACTD%f%f\n",pos_x,pos_y);
}

void vega_action_setAngle(float angle)
{
	uprintf(ACTION_USART,"ACTJ%f\n",angle);
}


void vega_action_init()
{
    HAL_Delay(10000);
    vega_action_reset();
}
