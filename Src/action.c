#include "action.h"
uint8_t send_data_action[8]={0};

/*************************************
*功能：全场定位清零
*说明：无
*************************************/
uint8_t action_clear()
{
    uprintf(ACTION_USART,"ACT0");
    return 1;
}

/*************************************
*功能;全场定位设置角度
*说明：action要求传入angle值设成浮点型，写
int因为进行位操作浮点型不支持。
*************************************/
uint8_t action_set_course_angle(UART_HandleTypeDef* USARTx,int angle)
{
    send_data_action[DATA_ACTION_1] = (uint8_t)(angle>>24);
    send_data_action[DATA_ACTION_2] = (uint8_t)(angle>>16);
    send_data_action[DATA_ACTION_3] = (uint8_t)(angle>>8);
    send_data_action[DATA_ACTION_4] = (uint8_t)(angle>>0);
    send_data_action[DATA_ACTION_5] = (uint8_t)'J';
    send_data_action[DATA_ACTION_6] = (uint8_t)'T';
    send_data_action[DATA_ACTION_7] = (uint8_t)'C';
    send_data_action[DATA_ACTION_8] = (uint8_t)'A';
    
    HAL_UART_Transmit(USARTx,(uint8_t *) send_data_action,sizeof(send_data_action),1000);
    return 1;
}
/*************************************
*功能：全场定位设置x坐标
*说明：action要求传入x值设成浮点型，写
int因为进行位操作浮点型不支持。
*************************************/
uint8_t action_set_x(UART_HandleTypeDef* USARTx,int x)
{
    send_data_action[DATA_ACTION_1] = (uint8_t)(x>>24);
    send_data_action[DATA_ACTION_2] = (uint8_t)(x>>16);
    send_data_action[DATA_ACTION_3] = (uint8_t)(x>>8);
    send_data_action[DATA_ACTION_4] = (uint8_t)(x>>0);
    send_data_action[DATA_ACTION_5] = (uint8_t)'X';
    send_data_action[DATA_ACTION_6] = (uint8_t)'T';
    send_data_action[DATA_ACTION_7] = (uint8_t)'C';
    send_data_action[DATA_ACTION_8] = (uint8_t)'A';
    
    HAL_UART_Transmit(USARTx,(uint8_t *) send_data_action,sizeof(send_data_action),1000);
    return 1;
}

/*************************************
*功能：全场定位设置y坐标
*说明：action要求传入y值设成浮点型，写
int因为进行位操作浮点型不支持。
*************************************/
uint8_t action_set_y(UART_HandleTypeDef* USARTx,int y)
{
    send_data_action[DATA_ACTION_1] = (uint8_t)(y>>24);
    send_data_action[DATA_ACTION_2] = (uint8_t)(y>>16);
    send_data_action[DATA_ACTION_3] = (uint8_t)(y>>8);
    send_data_action[DATA_ACTION_4] = (uint8_t)(y>>0);
    send_data_action[DATA_ACTION_5] = (uint8_t)'Y';
    send_data_action[DATA_ACTION_6] = (uint8_t)'T';
    send_data_action[DATA_ACTION_7] = (uint8_t)'C';
    send_data_action[DATA_ACTION_8] = (uint8_t)'A';
    
    HAL_UART_Transmit(USARTx,(uint8_t *) send_data_action,sizeof(send_data_action),1000);
    return 1;
}

/*************************************
*功能：全场定位更新坐标
*说明：底盘坐标跟新也写在这了
*************************************/
void action_updata()
{   
    if(pos_x<0)
        pos_x=0;
    if(pos_y<0)
        pos_y=0;
    chassis.pos_x = pos_x/1000;
    chassis.pos_y = pos_y/1000;
    chassis.angle = zangle*PI/180;
    
}

/*************************************
*功能：全场定位初始化
*说明：位置置0，角度置零，延时12s
*************************************/
void action_init()
{
    if(action_clear())
    {
      HAL_Delay(5000);//全场定位初始化要十一秒
      uprintf(CMD_USART,"Action init ok!!!");  
    }
}
