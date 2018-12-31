#ifndef __action_H
#define __action_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "math.h"
#include "chassis.h"
#include <usart.h>
#define ACTION_ID huart2
#define DATA_ACTION_8    0
#define DATA_ACTION_7    1
#define DATA_ACTION_6    2
#define DATA_ACTION_5    3
#define DATA_ACTION_4    4
#define DATA_ACTION_3    5
#define DATA_ACTION_2    6
#define DATA_ACTION_1    7
    
uint8_t action_clear();    
uint8_t action_set_course_angle(UART_HandleTypeDef* USARTx,int angle);    
uint8_t action_set_x(UART_HandleTypeDef* USARTx,int x);    
uint8_t action_set_y(UART_HandleTypeDef* USARTx,int y);    
void action_updata();
void action_init();
    
#ifdef __cplusplus
}
#endif
#endif /*__ action_H */