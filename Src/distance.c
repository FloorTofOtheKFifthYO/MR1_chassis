#include "distance.h"
int lock1=0;
int lock2=0;
int lock3=0;
int lock_laser=0;
double lockangle=0;
/**超聲波接收數據回調函數
*参数：can_rx_msg
*返回值： 无
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
        //uprintf("chassis.dis_1=%d\r\n",chassis.dis_1);
    }
    //memcpy((void*)dis_1,&temp.s32_form,4);
    else if(can_rx_msg->StdId == 0x08)
    {  
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_2 = temp.s32_form;
        temp.s32_form=0;
        //uprintf("chassis.dis_2=%d\r\n",chassis.dis_2);
        //memcpy((void*)dis_2,&temp.s32_form,4);
    }
    else if(can_rx_msg->StdId == 0x40)    
    {
        temp.u8_form[0] = can_rx_msg->Data[0];
        temp.u8_form[1] = can_rx_msg->Data[1];
        temp.u8_form[2] = can_rx_msg->Data[2];
        temp.u8_form[3] = can_rx_msg->Data[3];
        chassis.dis_3 = temp.s32_form;
        temp.s32_form=0;
        //uprintf("chassis.dis_3=%d\r\n",chassis.dis_3);
        //memcpy((void*)dis_3,&temp.s32_form,4);
    }
    
    
    
}

/**激光接受數據回調函數
*参数：can_rx_msg
*返回值： 无
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
        //uprintf("chassis.dis_laser=%d\r\n",chassis.dis_laser);
        //memcpy((void*)dis_laser,&temp.s32_form,4); 
    }
}

//int lock_angle(float angle)
//{
//  int error1=0,error_laser=0;
//  double turn=0;
//  error1=chassis.dis_1-dis1;
//  //error3=dis3-lock3;
//  error_laser=chassis.dis_laser-dislaser;
//  
//  uprintf("error1=%d\r\nerror_laser=%d\r\n",error1,error_laser);
//  turn = atan2((double)(error_laser - error1),(double)dis_bwt_laser_ul1);
//  uprintf("turn=%f",turn);
//    chassis_gostraight(0,0,turn);
//    
//    if(error2<0)
//      chassis_gostraight(500,0,0);
//    else
//      chassis_gostraight(500,3.1415926,0);
//  
//    if(error_laser<0)
//      chassis_gostraight(500,4.7123889,0);
//    else
//      chassis_gostraight(500,1.570796,0);
//   
//  if((turn - lock_turn)>0.0175||(turn - lock_turn)<-0.0175||abs(error2)>2||abs(error_laser)>2)
//    return 0;
//  else
//    return 1;
//}

/**鎖定預設距離
*参数：无
*返回值： 无
*/
void lock_distance(int lockx , int locky)
{
    
    int disx=0;
    int disy=0;
    
  loop:
    disx = chassis.dis_2 - lockx;
    disy = chassis.dis_laser - locky;
    
    uprintf("disx=%d\r\ndisy=%d\r\n",disx,disy);
    
    chassis_gostraight(200,atan2(disy,disx),0);
    
    if(abs(chassis.dis_2 - lockx) <= 5 && abs(chassis.dis_laser - locky) <= 5 )//&& (turn - lockangle) >= -0.0175 && (turn - lockangle) <= 0.0175)
    {
        return;
    }
    else 
    {
        goto loop;
    }
}

void lock_angle(float lockangle)
{
    float turnout=0;
    
  loop:    
    turnout = (chassis.dis_laser-lock_laser)-(chassis.dis_1 - lock1);
    chassis_gostraight(0,0,-turnout);
    
    if(abs((chassis.dis_laser-lock_laser)-(chassis.dis_1 - lock1))<=3)
    {
        return;
    }
    else 
    {
        goto loop;
    }
}

