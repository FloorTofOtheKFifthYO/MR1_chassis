#ifndef __cmd_func_H
#define __cmd_func_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "cmd.h"
#include "stdlib.h"
#include "maxon.h"
#include "math.h"
#include "chassis.h"
#include "vega.h"
#include "track.h"
#define PI 3.1415926535
void cmd_hello_func(int argc,char *argv[]);
void cmd_set_speed_func(int argc,char *argv[]);   
void cmd_stop_func(int argc,char *argv[]);   
void cmd_go_straight_func(int argc,char *argv[]);
void cmd_go_route_func(int argc, char *argv[]);
void cmd_reset_vega(int argc, char *argv[]);
void cmd_modify_angle(int argc, char *argv[]);
void cmd_angle_pid(int argc, char *argv[]);
void cmd_chassis_acc(int argc, char *argv[]);
void cmd_quickturn_pid(int argc, char *argv[]);
   
#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */