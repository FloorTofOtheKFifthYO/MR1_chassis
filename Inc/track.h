#ifndef __track_H
#define __track_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"
#include "bezier.h"    

#define NUM_POINTS 1000
#define input_num 7  //������Ƶ����
#define num 100     //�������Ƶ��Ĳ�������
#define K 1 //����������ϵ����kԽСԽ�ƽ����Ƶ㹹�ɵ�ֱ�ߣ���������������
#define out_num num*(input_num-1) //��������� 
typedef struct
{
    float X;
    float Y;
} PointF;
    
extern float chassis_xpos[NUM_POINTS];//X���� 
extern float chassis_ypos[NUM_POINTS];//Y���� 
extern float chassis_turn[NUM_POINTS];
extern float chassis_speed_max;
extern PointF in[input_num];
extern int chassis_posnum;
extern void track_init();
    
    
    
    
    
    
    
        
#ifdef __cplusplus
}
#endif
#endif /*__ track_H */