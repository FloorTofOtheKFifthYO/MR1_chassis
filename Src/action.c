#include "action.h"
uint8_t send_data_action[8]={0};

/*************************************
*���ܣ�ȫ����λ����
*˵������
*************************************/
uint8_t action_clear()
{
    uprintf(ACTION_USART,"ACT0");
    return 1;
}

/*************************************
*����;ȫ����λ���ýǶ�
*˵����actionҪ����angleֵ��ɸ����ͣ�д
int��Ϊ����λ���������Ͳ�֧�֡�
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
*���ܣ�ȫ����λ����x����
*˵����actionҪ����xֵ��ɸ����ͣ�д
int��Ϊ����λ���������Ͳ�֧�֡�
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
*���ܣ�ȫ����λ����y����
*˵����actionҪ����yֵ��ɸ����ͣ�д
int��Ϊ����λ���������Ͳ�֧�֡�
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
*���ܣ�ȫ����λ��������
*˵���������������Ҳд������
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
*���ܣ�ȫ����λ��ʼ��
*˵����λ����0���Ƕ����㣬��ʱ12s
*************************************/
void action_init()
{
    if(action_clear())
    {
      HAL_Delay(5000);//ȫ����λ��ʼ��Ҫʮһ��
      uprintf(CMD_USART,"Action init ok!!!");  
    }
}
