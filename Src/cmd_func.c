#include "cmd_func.h"


void cmd_hello_func(int argc,char *argv[])
{
    uprintf("hello world");
}
void cmd_set_speed_func(int argc,char *argv[])//setspeed 1000 1000 1000
{
    maxon_setSpeed(&MOTOR0_ID,atoi(argv[1]));
    maxon_setSpeed(&MOTOR1_ID,atoi(argv[2]));
    maxon_setSpeed(&MOTOR3_ID,atoi(argv[3])); 
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

void cmd_go_route_func(int argc, char *argv[])
{
    chassis_go_route(atoi(argv[1]));
}

void cmd_reset_vega(int argc, char *argv[])
{
    vega_reset();
}

void cmd_modify_angle(int argc, char *argv[])
{
    ERR_angle_m0 = atof(argv[1]) + 2*2*PI/3;
    uprintf("ERR_angle_m0 = %f + 2*2*PI/3 \r\n", atof(argv[1]));
    ERR_angle_m1 = atof(argv[2]) + 1*2*PI/3;
    uprintf("ERR_angle_m1 = %f + 1*2*PI/3 \r\n", atof(argv[2]));
}

void cmd_angle_pid(int argc, char *argv[])
{
    Angle_KP = atof(argv[1]);
    Angle_KD = atof(argv[2]);
}
