#ifndef __distance_H
#define __distance_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"
#include "chassis.h"
#include "stdlib.h"  
  
#define ultrason1_id 0x08
#define ultrason2_id 0x10
#define ultrason3_id 0x40
#define laser_id 0x66
#define dis_bwt_laser_ul1 75  //测平行传感器间距离
 
extern int lock1;
extern int lock2;
extern int lock3;
extern int lock_laser;
extern double lockangle;
extern int lock_flag;
    

void ultrasonic_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg);
void laser_msg_rev_callback(CanRxMsgTypeDef *can_rx_msg); 
void read_distance(uint16_t id);
void read_alldis(); 
void lock_distance(int lockx , int locky);
void lock_angle(float lockangle);
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#ifdef __cplusplus
}
#endif
#endif /*__ distance_H */