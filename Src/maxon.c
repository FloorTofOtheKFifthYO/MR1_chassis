/*******************************************************************************
Copyright:      2018/12/18
File name:      maxon.c
Description:    maxon电机控制，设置速度，设置PI系数，保存设置系数
Author:         徐铭远，王云轩
Version：       1.0
Data:           2018/12/18 22:36
History:        无
*******************************************************************************/
#include "maxon.h"


//速度发送协议，第三个元素为节点，最后一个为校验位
uint8_t sendToSetSpeed[10] = {0x55,0xAA,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00};
uint8_t sendToSetSpeedPI[10] = {0x55,0xAA,0x01,0x00,0x84,0x00,0x00,0x00,0x00,0x00};

/**maxon电机设置速度
*参数：串口号 速度值
*返回值： 无
*/
void maxon_setSpeed(UART_HandleTypeDef* USARTx, int speed){
    int i;
    int check = 0;
    
    sendToSetSpeed[NUMBER] = 0x01;
    sendToSetSpeed[DATA_4] = (uint8_t)(speed>>24);
    sendToSetSpeed[DATA_3] = (uint8_t)(speed>>16);
    sendToSetSpeed[DATA_2] = (uint8_t)(speed>>8);
    sendToSetSpeed[DATA_1] = (uint8_t)(speed>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeed[i];
    }
    sendToSetSpeed[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeed,sizeof(sendToSetSpeed),1000);	
}

//p:7000 i:4000

/**maxon电机设置KP
*参数：串口号 P值
*返回值： 无
*/
void maxon_setSpeed_p(UART_HandleTypeDef* USARTx , int p){
    int i;
    int check = 0;
    sendToSetSpeedPI[5] = 0x02;//命令偏移量
    sendToSetSpeedPI[6] = 0x02;//数据长度
    sendToSetSpeedPI[7] = (uint8_t)(p>>8);
    sendToSetSpeedPI[8] = (uint8_t)(p>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeedPI[i];
    }
    sendToSetSpeedPI[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeedPI,sizeof(sendToSetSpeedPI),1000);
    
}

/**maxon电机设置KI
*参数：串口号 KI
*返回值： 无
*/
void maxon_setSpeed_i(UART_HandleTypeDef* USARTx , int si){
    int i;
    int check = 0;
    
    sendToSetSpeedPI[5] = 0x03;//命令偏移量
    sendToSetSpeedPI[6] = 0x02;//数据长度
    sendToSetSpeedPI[7] = (uint8_t)(si>>8);
    sendToSetSpeedPI[8] = (uint8_t)(si>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeedPI[i];
    }
    sendToSetSpeedPI[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeedPI,sizeof(sendToSetSpeedPI),1000);
}

/**maxon电机保存参数
*参数：串口号
*返回值： 无
*/
void maxon_save(UART_HandleTypeDef* USARTx){
    uint8_t saveinfo[8] = {0x55 , 0xAA , 0x01 , 0x00 , 0x03 , 0x01 ,0x00 , 0x04};
    HAL_UART_Transmit(USARTx,(uint8_t *) saveinfo,sizeof(saveinfo),1000);
}
