#ifndef __vavle_H
#define __vavle_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"    
    
    
void vavle_open(uint8_t id);
void vavle_close(uint8_t id);
void vavle_control();
    
    
    
    
    
    
    
    
#ifdef __cplusplus
}
#endif
#endif /*__ vavle_H */