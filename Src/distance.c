/**
  ******************************************************************************
  * File Name          : DISTANCE.C
  * ����               ��void ultrasonic_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg)
  *                      void laser_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg)
  *                      void lock_distance(int lockx , int locky)
  *                      void lock_angle(int laser,int ultrasonic)
  * �h��               : ȡ���x���{�����Լ����xȡ�ľ��xֵ�M��̎��
  *
  ******************************************************************************
*/
#include "distance.h"

int lock1=0;
int lock2=0;
int lock3=0;
int lock_laser=0;

/**�������Ք������{����
*������can_rx_msg
*����ֵ�� ��
*/
void ultrasonic_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg){
    data_convert temp;
    if(can_rx_msg->StdId == 0x10)
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_1 = temp.s32_form;
        temp.s32_form=0;
    }
    else if(can_rx_msg->StdId == 0x08)
    {  
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_2 = temp.s32_form;
        temp.s32_form=0;
    }
    else if(can_rx_msg->StdId == 0x40)    
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_3 = temp.s32_form;
        temp.s32_form=0;
    } 
}

/**������ܔ������{����
*������can_rx_msg
*����ֵ�� ��
*/
void laser_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg){
    data_convert temp;
    if(can_rx_msg->StdId==0x66)
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_laser = temp.s32_form;
        temp.s32_form=0;
    }
}

/**�i���A�O���x
*������y�S�ɼ����ʾ���x��x�S�Ɍ���������ʾ���x
*����ֵ�� ��
*/
void lock_distance(int lockx , int locky)
{
    
    int disx=0;//x����ƫ��
    int disy=0;//y����ƫ��
    
  loop:
    disx = chassis.dis_2 - lockx;
    disy = chassis.dis_laser - locky;
    
    uprintf("disx=%d\r\ndisy=%d\r\n",disx,disy);
    
    chassis_gostraight(200,atan2(disy,disx),0);
    
    if(abs(chassis.dis_2 - lockx) <= 5 && abs(chassis.dis_laser - locky) <= 5 )
    {
        return;
    }
    else 
    {
        goto loop;
    }
}

/**�i���A�O�Ƕȣ���ƽ�е��A�O�����ͼ�����xֵ�ж��Ƿ�ͳ�ʼ��Bƽ��
*�������A�O���x���⣬����
*����ֵ�� ��
*/
void lock_angle(int laser,int ultrasonic)
{
    float turnout=0;
    
  loop:    
    turnout = (chassis.dis_laser-laser)-(chassis.dis_1 - ultrasonic);
    chassis_gostraight(0,0,-turnout);
    
    if(abs((chassis.dis_laser-laser)-(chassis.dis_1 - ultrasonic))<=3)
    {
        return;
    }
    else 
    {
        goto loop;
    }
}

