#ifndef __chassis_H
#define __chassis_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "vega.h"
#include "maxon.h"
#include "usart.h"    
#include "math.h"  
#include "track.h"
#include <stdbool.h>
#include "distance.h"
    
    
#define PI 3.1415926535
#define EXP 2.718281828
//#define NUM_POINTS 100
typedef struct
{
	//全场定位传回的数据
	int g_vega_pos_x; 
	int g_vega_pos_y;   
	float g_vega_angle;
	
	float radar_theta;
	float radar_distance;
	
	float pos_x,pos_y;
	float angle;
	
	float temp_angle;
	
	//状态
	enum {car_stop, car_running, car_ready} car_state;//车的运动状态
	
	int dis_1;
        int dis_2;
        int dis_3;
        int dis_laser;
	//参数
	float Angle_radium;//停角度范围
	int Angle_speed;
	float factor;
	float Move_radium;
	int Move_speed;
	int Speed_max;
	int Speed_min;
	float Start_distance;
	float xfactor;
} Chassis;  

//extern double chassis_speed_max ;
extern float ERR_angle_m3 , ERR_angle_m1 , ERR_angle_m0  ;
extern float chassis_turn_angle_KP;
extern float chassis_turn_angle_KD;
extern int g_ispeed;
extern float g_fangle;
extern float g_fturn;
//extern double chassis_speed[NUM_POINTS];
extern double param_a;
extern double param_b;
extern Chassis chassis;
void chassis_init(void);
void chassis_update(void);
void chassis_gostraight(int speed , float angle, float turn); 
void chassis_calculate_dis_matrix();
extern void chassis_exe();
extern void chassis_go_route(int);

    
    
    
    
    
    
    
    
    
    
#ifdef __cplusplus
}
#endif
#endif /*__ chassis_H */