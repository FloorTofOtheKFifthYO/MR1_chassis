/*******************************************************************************
Copyright:      2018/12/18
File name:      cmd_func.c
Description:    存放串口命令函数，用户自己添加，要求设成外部可以调用
Author:         徐铭远
Version：       1.0
Data:           2018/12/18 22:36
History:        无
Bug:            无
*******************************************************************************/
#include "cmd_func.h"

void cmd_hello_func(int argc,char *argv[])
{
    uprintf(CMD_USART,"hello world");
}
void cmd_stop_func(int argc,char *argv[])
{
    maxon_setSpeed(&MOTOR0_ID,0);
    maxon_setSpeed(&MOTOR3_ID,0);
    maxon_setSpeed(&MOTOR1_ID,0); 
}
void cmd_go_straight_func(int argc,char *argv[])//go_straight 1000 30
{
    //chassis_gostraight( atoi(argv[1]) , atof(argv[2]) ,atof(argv[3]));
    g_ispeed = atoi(argv[1]);
    g_fangle = atof(argv[2]);
    g_fturn = atof(argv[3]);
}

void cmd_go_route_func(int argc, char *argv[])//go_route 1 5000 1 2500
{
    chassis_go_route(atoi(argv[1]));
    param_a = atof(argv[2]);
    param_b = atof(argv[3]);
    if(atof(argv[4]) > 0)
    {
        chassis_speed_max = atof(argv[4]);  
    }
    
}

void cmd_reset_vega(int argc, char *argv[])
{
    vega_action_reset();
}

void cmd_modify_angle(int argc, char *argv[])
{
    ERR_angle_m0 = atof(argv[1]) + 2*2*PI/3;
    uprintf(CMD_USART,"ERR_angle_m0 = %f + 2*2*PI/3 \r\n", atof(argv[1]));
    ERR_angle_m1 = atof(argv[2]) + 1*2*PI/3;
    uprintf(CMD_USART,"ERR_angle_m1 = %f + 1*2*PI/3 \r\n", atof(argv[2]));
}

void cmd_angle_pid(int argc, char *argv[])
{
    chassis_turn_angle_KP = atof(argv[1]);
    chassis_turn_angle_KD = atof(argv[2]);
    
}

void cmd_print_pos(int argc, char *argv[])
{
    uprintf(CMD_USART,"x = %f, y = %f\r\n",chassis.pos_x,chassis.pos_y);
}

void cmd_go_targetpoint(int argc, char *argv[])//go_point x y
{
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float angle = atan2(y-chassis.g_vega_pos_y*0.001,x-chassis.g_vega_pos_x*0.001);
    //uprintf(CMD_USART,"angle=%f\r\n",angle);
    while(!((y-chassis.g_vega_pos_y*0.001)<0.1&&(y-chassis.g_vega_pos_y*0.001)>-0.1&&(x-chassis.g_vega_pos_x*0.001)<0.1&&(x-chassis.g_vega_pos_x*0.001)>-0.1))
    {
        chassis_gostraight(1000 , angle, 0);
        //uprintf(CMD_USART,"x = %f, y = %f\r\n",chassis.g_vega_pos_x*0.001,chassis.g_vega_pos_y*0.001);
    }
    chassis_gostraight(0,0,0);
}

void cmd_read_distance(int argc, char *argv[])//read 1
{
    if(atoi(argv[1])==1)
    {
        can_send_msg(0x66,"open",4);
        while(chassis.dis_1==0||chassis.dis_2==0||chassis.dis_laser1==0||chassis.dis_laser2==0);
        uprintf(CMD_USART,"start read distance from sensor!\r\n");
        float u1=chassis.dis_1;
        float u2=chassis.dis_2;
        float l1=chassis.dis_laser1;
        float l2=chassis.dis_laser2;
        uprintf(CMD_USART,"u1 = %f  u2 = %f laser1 = %f laser2 = %f\r\n",u1,u2,l1,l2);
    }
    else
    {
        can_send_msg(0x66,"close",5);
        uprintf(CMD_USART,"stop read distance from sensor!\r\n"); 
    }
}

void cmd_lock_x_y_angle(int argc, char *argv[])//lockx x y angle
{
    Lock_angle(atof(argv[1]));
    Lock_X(atof(argv[2]));
    Lock_Y(atof(argv[3]));
}
