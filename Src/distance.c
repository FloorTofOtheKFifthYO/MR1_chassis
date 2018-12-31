/*******************************************************************************
Copyright:      2018/12/18
File name:      distance.c
Description:    传感器测距辅助定位模块,要先can发送下面两个命令开关，就会更新到低
底盘坐标can_send_msg(0x66,"open",4);can_send_msg(0x66,"close",5);
Author:         徐铭远
Version：       1.0
Data:           2018/12/18 22:36
History:        无
Bug:            无
*******************************************************************************/
#include "distance.h"
int lock1=0;
int lock2=0;
int lock_laser1=0;
int lock_laser2=0;
double lockangle=0;
/**超聲波接收數據回調函數
*参数：can_rx_msg
*返回值： 无
*/
void ultrasonic_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg){
    data_convert temp;
    if(can_rx_msg->StdId == 0x40)
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_1 = temp.s32_form;
        temp.s32_form=0;
        //uprintf(CMD_USART,"chassis.dis_1=%d\r\n",chassis.dis_1);
    }
    //memcpy((void*)dis_1,&temp.s32_form,4);
    else if(can_rx_msg->StdId == 0x66)
    {  
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_2 = temp.s32_form;
        temp.s32_form=0;
        //uprintf(CMD_USART,"chassis.dis_2=%d\r\n",chassis.dis_2);
        //memcpy((void*)dis_2,&temp.s32_form,4);
    }
}

/**激光接受數據回調函數
*参数：can_rx_msg
*返回值： 无
*/
void laser_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg){
    data_convert temp;
    if(can_rx_msg->StdId==0x08)
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_laser1 = temp.s32_form;
        temp.s32_form=0;
        //uprintf(CMD_USART,"chassis.dis_laser=%d\r\n",chassis.dis_laser1);
        //memcpy((void*)dis_laser,&temp.s32_form,4); 
    }else if(can_rx_msg->StdId==0x10)
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_laser2 = temp.s32_form;
        temp.s32_form=0;
        //uprintf(CMD_USART,"chassis.dis_laser=%d\r\n",chassis.dis_laser2);
        //memcpy((void*)dis_laser,&temp.s32_form,4); 
    }
}

uint8_t Lock_X(float target)
{
    float speed;
    float error,l_error;
    while((target-X_Sensor)<-5||(target-X_Sensor)>5)
    {
        error = target - X_Sensor;
        speed = 2 * error + 0.6 * (error - l_error);
        chassis_gostraight((int)speed,0,0);
        l_error = error;
    }
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    HAL_Delay(10);
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    return 1;
}

uint8_t Lock_Y(float target)
{
    float speed;
    float error,l_error;
    while((target-Y_Sensor)<-3||(target-Y_Sensor)>3)
    {
        error = target - Y_Sensor;
        speed = 4.5 * error + 0.6 * (error - l_error);
        chassis_gostraight((int)speed,PI/2,0);
        l_error = error;
    }
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    HAL_Delay(10);
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    return 1;
}

uint8_t Lock_angle(float delt)
{
    float angle;
    float error,l_error;
    while((chassis.dis_laser1-chassis.dis_laser2-delt)<-3||(chassis.dis_laser1-chassis.dis_laser2-delt)>3)
    {
        error = chassis.dis_laser1 - chassis.dis_laser2-delt;
        angle = 0.005 * error + 0.01 * (error - l_error);
        chassis_gostraight(0,0,-angle);
        l_error = error;
    }
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    HAL_Delay(10);
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0);
    return 1;
}