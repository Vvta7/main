/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
    uint8_t flag_function,flag_function2;//为什么有两个，因为按键抬起时会抖动，又会消除标志位，所以设两个，互不影响
    int iii=0;
    int ccr_number=498;//范围0~1000，每次变化83，83*12=996
//    int expect_ccr[12]={0};//存放12个档位对应的ccr值,保证周围光线越弱，占空比越高
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint16_t ADC_value;		//ADC采集到的数据
    float  volt_value;	//电压值

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//oled
    delay_init(72);
    OLED_Init();
    OLED_Clear();
//PWM
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//这里是100HZ，PWM2模式，先低后高,这个时候设置占空比40%，实际高电平60%
    //    __HAL_TIM_SET_COUNTER(&htim3,0);//设置计数器的值,
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,498);//设置占空比50%,

////ADC
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,10);
//////time
    HAL_TIM_Base_Start_IT(&htim4);//打开定时器4定时中断
//////////
        delay_ms(300);
//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      OLED_Clear();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    __HAL_TIM_SET_COUNTER(&htim3,0);//设置计数器的值,

        OLED_ShowNum(0,1,flag_function,1,12);//数字
        ADC_value = ADC_X_Get(10);
        OLED_ShowNum(60,0,ADC_value,4,12);//数字
        OLED_ShowNum(0,0,ccr_number,4,12);//数字
      
        if(flag_function==2)//感光调光模式
        {
            ADC_value = ADC_X_Get(10);
            ccr_number=999-ADC_value/4000.0*999;
            __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ccr_number);//设置占空比,
            OLED_ShowNum(60,0,ADC_value,4,12);//数字
            
        }
        else if(flag_function==4)//自发调光模式
        {
            for(u8 i=0;i<12;i++)
                {
                    ccr_number=ccr_number-83;
                    if(ccr_number<50){ccr_number=0;}
                    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ccr_number);//设置占空比,
                    
                    delay_ms(1000);
                    if(ccr_number==0){break;}
                }
            for(u8 i=0;i<12;i++)
                {
                    ccr_number=ccr_number+83;
                    if(ccr_number>=996){ccr_number=999;}
                    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ccr_number);//设置占空比,
                    
                    delay_ms(1000);
                    if(ccr_number>=999){break;}
                }
        }
        else if(flag_function==1)//手动调光模式减
        {   
            ccr_number=ccr_number-83;
            if(ccr_number<80){__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);ccr_number=0;}
            else 
            {__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ccr_number);}//设置占空比,

        }
        else if(flag_function==3)//手动调光模式增
        {   
            ccr_number=ccr_number+83;
            if(ccr_number>=996){__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,999);ccr_number=999;}
            else 
                {__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ccr_number);//设置占空比,
                }
        
        }
        flag_function=0;//消除标志位
        delay_ms(300);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
     uint8_t flag_Time=time_real[0];
    
    if(GPIO_Pin == KEY1_Pin)            //如果发生外部中断的是KEY_Pin
    {
        if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==0)
        {flag_function=1;
        while(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET)//等待按键松开，避免LED状态重复翻转
        {
        if(time_real[0]-flag_Time>3)
        {   OLED_ShowNum(0,3,time_real[0]-flag_Time,2,12);//数字
            flag_function=2;
        }
        }
    }}
    else if(GPIO_Pin == KEY2_Pin)            //如果发生外部中断的是KEY_Pin
    {
        if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==0)
        {flag_function=3;
        while(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET)//等待按键松开，避免LED状态重复翻转
        {
        if(time_real[0]-flag_Time>3)
        {OLED_ShowNum(0,3,time_real[0]-flag_Time,2,12);//数字
            flag_function=4;
        }
        }
    }}
    
    if(GPIO_Pin == zero_check_Pin)            //如果发生外部中断的是zero_check_Pin,高电平进入
    {
       if(HAL_GPIO_ReadPin(zero_check_GPIO_Port,zero_check_Pin)==1)
       {iii++;
        __HAL_TIM_SET_COUNTER(&htim3,0);//设置计数器的值
 //       HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
 //       HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    }}
    OLED_ShowNum(0,1,flag_function,1,12);//数字

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
