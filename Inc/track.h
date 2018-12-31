#ifndef __track_H
#define __track_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"
#include "bezier.h"    

#define NUM_POINTS 1000
#define input_num 7  //输入控制点个数
#define num 100     //两个控制点间的采样个数
#define K 1 //贝塞尔曲线系数，k越小越逼近控制点构成的直线，但仍是连续曲线
#define out_num num*(input_num-1) //总输出点数 
typedef struct
{
    float X;
    float Y;
} PointF;
    
extern float chassis_xpos[NUM_POINTS];//X坐标 
extern float chassis_ypos[NUM_POINTS];//Y坐标 
extern float chassis_turn[NUM_POINTS];
extern float chassis_speed_max;
extern PointF in[input_num];
extern int chassis_posnum;
extern void track_init();
    
    
    
    
    
    
    
        
#ifdef __cplusplus
}
#endif
#endif /*__ track_H */