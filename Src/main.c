/*******************************************************************************
Copyright:      2018/12/18
File name:      main.c
Description:    程序入口，包含系统时钟配置，外设初始化。
Author:         徐铭远，王云轩
Version：       底盘贝塞尔曲线+action全场定位
Data:           2018/12/18 22:36
History:        删去vega.c文件
*******************************************************************************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "maxon.h"
#include "cmd.h"
#include "cmd_func.h"
#include "chassis.h"
#include "vega.h"
#include "dma.h"

int main_flag = 0;
int chassis_flag = 0;

void SystemClock_Config(void);

void main()
{
    HAL_Init();
    SystemClock_Config();    
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_TIM1_Init();
    MX_UART5_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();
    MX_TIM2_Init();
    
    
    cmd_init();
    can_init(); 
    chassis_init();
    
    uprintf(CMD_USART,"start...\r\n");
    main_flag = 1;
    usart_init();//在里面开串口接收中断
    while (1)
    {
        //uprintf(CMD_USART,"start...\r\n");
        usart_exc();
        if(chassis_flag == 1)
        {
            chassis_exe();
            chassis_flag = 0;
        }
    }
    
    
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    
    /**Configure the main internal regulator output voltage 
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    
    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void){
    static int time_1ms_cnt;
    time_1ms_cnt++;
    if( main_flag == 1)
    {
        if(time_1ms_cnt % 10 == 0)
        { 
            chassis_flag = 1;
        }
        if(time_1ms_cnt % 50 == 0)
        {
            
        }
        if(time_1ms_cnt % 500 == 0)
        {
            
        }
        if(time_1ms_cnt >= 65533)
        {
            time_1ms_cnt = 0;
        }
        
    }
}

/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
        uprintf(CMD_USART,"程序废了，老哥，赶紧复位！！！！！！！\r\n");
    }
    /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
    
}

#endif

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
