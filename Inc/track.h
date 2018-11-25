#ifndef __track_H
#define __track_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"
#define NUM_POINTS 3000
extern double chassis_xpos[NUM_POINTS];//X×ø±ê 
extern double chassis_ypos[NUM_POINTS];//Y×ø±ê 
extern double chassis_turn[NUM_POINTS];
extern double chassis_speed_max;
extern int chassis_posnum;
extern void track_init();
    
    
    
    
    
    
    
    
        
#ifdef __cplusplus
}
#endif
#endif /*__ track_H */