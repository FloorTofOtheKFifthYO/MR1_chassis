/*******************************************************************************
Copyright:      2018/12/18
File name:      can.c
Description:    can����ͨ�ţ�canϵͳ���ã�can��ʼ����can���������ã�can���ͺ���
                can��ӻص���can�����б�ƥ�䣬can���ջص���can����ص�
Author:         ������
Version��       1.0
Data:           2018/12/18 22:36
History:        ��
Bug:            ���չ���can���
*******************************************************************************/
#include "can.h"
#include <assert.h>
#include "gpio.h"
#include "vega.h"
#include "distance.h"

/* USER CODE BEGIN 0 */
static int canlistnum = 0;
CanList canList[50];//����ܼ�50��can���ӣ����Ը�
CAN_FilterConfTypeDef  sFilterConfig;
static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;

void Configure_Filter(void);
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_9TQ;
  hcan1.Init.BS2 = CAN_BS2_4TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 



/* USER CODE BEGIN 1 */
/****
    *@brief can��ʼ��
****/
void can_init()
{
    hcan1.pTxMsg = &TxMessage;
    hcan1.pRxMsg = &RxMessage;
    Configure_Filter();
    if(HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0)!=HAL_OK)
    {
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FOV0 | CAN_IT_FMP0);
    }
    uprintf(CMD_USART,"can ready!!!");
      //can_add_callback(0X11,vega_msg_rcv_callback);//��ӽ���vega��Ϣ�Ļص�����
      can_add_callback(0x08,laser_msg_rev_callback);//�������
      can_add_callback(0x10,laser_msg_rev_callback);//�������
      can_add_callback(0x40,ultrasonic_msg_rev_callback);//������
      can_add_callback(0x66,ultrasonic_msg_rev_callback);//������
}

void Configure_Filter(void)
{  
    sFilterConfig.FilterNumber = 0;                   //��������0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //�����ڱ�ʶ������λģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//�˲���λ��Ϊ����32λ
	
	
    sFilterConfig.FilterIdHigh =0x0000<<5;//0x0122<<5;//(((unsigned int)0x1314<<3)&0xFFFF0000)>>16; //Ҫ���˵ģɣĸ�λ
    sFilterConfig.FilterIdLow = 0x0000;//(((unsigned int)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;//Ҫ���˵�ID��λ
    sFilterConfig.FilterMaskIdHigh =0x0000;// 0xffff;
    sFilterConfig.FilterMaskIdLow = 0x0000;//0xffff;
    sFilterConfig.FilterFIFOAssignment =CAN_FILTER_FIFO0;//��������������FIFO0��
    sFilterConfig.FilterActivation = ENABLE;//ʹ�ܹ�����
    sFilterConfig.BankNumber = 14;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}



/****
    *@brief ��can���߷�������
    *@param ID : �������ݵ�ID
    *@param data : ���͵����ݣ�����Ϊ8
    *@retval : ����ʧ�ܷ���0����������1
*/
int can_send_msg(uint32_t ID, uint8_t* data, uint32_t len)
{
    hcan1.pTxMsg->StdId = ID;
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;
    hcan1.pTxMsg->IDE = CAN_ID_STD;
    hcan1.pTxMsg->DLC = len;
    for(int i = 0; i < 8; i++)
    {
        hcan1.pTxMsg->Data[i] = data[i];
    }
    if(HAL_CAN_Transmit(&hcan1, 100) != HAL_OK) 
    {
         return 0;
    }
    return 1;
}

/****
    *@brief ��can���߽��ձ��������
    *@param ID : �������ݵ�ID
    *@param void (*func)(uint8_t data[8]) : �����������ӵĴ�����
    *@retval ����1��ʾIDԽ�磬����0��ʾ����
****/
int can_add_callback(uint32_t ID, void (*func)(CanRxMsgTypeDef* pRxMsg)) 
{
    assert(ID <= 2048);
    assert(canlistnum < 50);
    canList[canlistnum].func = func;
    canList[canlistnum].ID = ID;
    canlistnum++;
    return 0;
}


/****
    *@brief �����յ���������can���߽��ձ�ƥ��
    *@param ID : ���յ����ݵ�ID
    *@param uint8_t data[8] : ���յ�����
    *@retval ����1��ʾƥ��ʧ�ܣ�����0��ʾ����
*/
int CAN_LIST_MATCH(uint32_t ID, CanRxMsgTypeDef* pRxMsg)
{
    for(int i = 0; i < canlistnum; i++)
    {
        if(ID == canList[i].ID)
        {
            (*canList[i].func)(pRxMsg);
            return 1;
        }
    }
    return 0;
}


/****
    *@brief can���ջص�����
****/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan1)	
{
    RxMessage.ExtId=hcan1->pRxMsg->StdId;
    CAN_LIST_MATCH(hcan1->pRxMsg->StdId, hcan1->pRxMsg);
    if(HAL_CAN_Receive_IT(hcan1,CAN_FIFO0)!=HAL_OK)
    {
        __HAL_CAN_ENABLE_IT(hcan1, CAN_IT_FOV0 | CAN_IT_FMP0);
    }
}

/****
    *@brief can����ص�
****/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){

    hcan1.Instance->MSR=0;
    if(HAL_CAN_Receive_IT(hcan,CAN_FIFO0)!=HAL_OK)
    {
        __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FOV0 | CAN_IT_FMP0);
    }
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
