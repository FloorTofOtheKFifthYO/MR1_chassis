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
#include "distance.h"


     
void cmd_hello_func(int argc,char *argv[]); 
void cmd_stop_func(int argc,char *argv[]);   
void cmd_go_straight_func(int argc,char *argv[]);
void cmd_go_route_func(int argc, char *argv[]);
void cmd_reset_vega(int argc, char *argv[]);
void cmd_modify_angle(int argc, char *argv[]);
void cmd_angle_pid(int argc, char *argv[]);
void cmd_print_pos(int argc, char *argv[]);
void cmd_go_targetpoint(int argc, char *argv[]);//go_point x y
void cmd_read_distance(int argc, char *argv[]);
void cmd_lock_x_y_angle(int argc, char *argv[]);
#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */