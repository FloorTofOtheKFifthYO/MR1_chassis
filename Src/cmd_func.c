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

void cmd_go_route_func(int argc, char *argv[])//go_route 1 1500
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
  chassis_turn_angle_KP = atof(argv[1]);
  chassis_turn_angle_KD = atof(argv[2]);
  
}
void cmd_chassis_acc(int argc, char *argv[])//start_acc 1 1500 1
{
  //start_flag=atoi(argv[1]);
}

void cmd_lock_distance(int argc, char *argv[])//lock_distance
{
  can_send_msg(0x66,"open",4);
  lock_angle(lockangle);
  lock_distance(lock2,lock_laser);
  can_send_msg(0x66,"close",5);
  uprintf("lock ok!!!");
}

void cmd_check_distance(int argc, char *argv[])//check_distance
{
  chassis.dis_1=0;
  chassis.dis_2=0;
  chassis.dis_3=0;
  chassis.dis_laser=0;
  
  can_send_msg(0x66,"open",4);
  while(chassis.dis_1==0||chassis.dis_2==0||chassis.dis_3==0||chassis.dis_laser==0);
  can_send_msg(0x66,"close",5);
  
  lock1=chassis.dis_1;
  lock2=chassis.dis_2;
  lock3=chassis.dis_3;
  lock_laser=chassis.dis_laser;
  lockangle=atan2((double)(lock_laser - lock1),(double)dis_bwt_laser_ul1);
  uprintf("check ok!!!\r\nultrason1=%d\r\nultrason2=%d\r\nultrason3=%d\r\nlaser=%d\r\nturn=%f\r\n",chassis.dis_1,chassis.dis_2,chassis.dis_3,chassis.dis_laser,lockangle);
}