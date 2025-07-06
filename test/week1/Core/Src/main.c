/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile struct {
  uint16_t exti_pin;      // 触发中断的引脚
  GPIO_TypeDef* port;     // 引脚端口
  uint8_t active;         // 消抖进行中标志
} debounce_info={0};
char message[50];
char receivedata[50];
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void datapro(char data[],int len);
void pwm_out(int n,int key);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin==key1_Pin)
  {
    debounce_info.active = 1;
    debounce_info.exti_pin = GPIO_Pin;
    debounce_info.port=key1_GPIO_Port;
    HAL_TIM_Base_Start_IT(&htim2);
  }
  if(GPIO_Pin==key2_Pin)
  {
    debounce_info.active = 1;
    debounce_info.exti_pin = GPIO_Pin;
    debounce_info.port=key2_GPIO_Port;
    HAL_TIM_Base_Start_IT(&htim2);
  }
}

//定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim==(&htim2)&&debounce_info.active==1)
  {
    HAL_TIM_Base_Stop_IT(&htim2);
    if(HAL_GPIO_ReadPin(debounce_info.port,debounce_info.exti_pin)==0) {
      if (debounce_info.exti_pin==key1_Pin) {
        HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
      }
      if (debounce_info.exti_pin==key2_Pin) {
        HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
      }

    }
    debounce_info.active=0;
  }
  // if (htim==(&htim3)) {
  //   char led_1[5]={0},led_2[5]={0},status[50]={0};
  //   if (HAL_GPIO_ReadPin(led1_GPIO_Port, led1_Pin)==0)
  //      strcpy(led_1,"low");
  //   else
  //     strcpy(led_1,"high");
  //   if (HAL_GPIO_ReadPin(led2_GPIO_Port, led2_Pin)==0)
  //     strcpy(led_2,"low");
  //   else
  //     strcpy(led_2,"high");
  //   sprintf(status,"led1:%s   led2:%s",led_1,led_2);
  //   HAL_UART_Transmit_DMA(&huart1,status,strlen(status));
  // }
}
//接收完成
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart==&huart1) {
    datapro(receivedata,strlen(receivedata));
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message, strlen(message));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)receivedata, sizeof(receivedata));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
  }
}
void datapro(char data[],int len) {
  if (len==10) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  }
  else if (len==12) {
    if (data[8]-'1'==0)
      pwm_out(1,data[9]-'0');
    if (data[8]-'2'==0) {
      pwm_out(2,data[9]-'0');
    }
  }
  else if (len==14) {
    if (data[8]-'1'==0) {
      pwm_out(1,data[9]-'0');
      pwm_out(2,data[11]-'0');
    }
    else if (data[8]-'2'==0) {
      pwm_out(2,data[9]-'0');
      pwm_out(8,data[11]-'0');
    }
  }
}
void pwm_out(int n,int key) {
  if (n==1) {
    switch (key) {
      case 0:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        break;
      case 1:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 24);
        break;
      case 2:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 49);
        break;
      case 3:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 74);
        break;
      case 4:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 99);
        break;
    }
  }
  else if (n==2) {
    switch (key) {
      case 0:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        break;
      case 1:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 24);
        break;
      case 2:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 49);
        break;
      case 3:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 74);
        break;
      case 4:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 99);
        break;
    }
  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,receivedata,sizeof(receivedata));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);//关闭过半中断
  HAL_TIM_Base_Start_IT(&htim3);//每隔0.5s发送一次状态
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  float you=3.1415;
  int i=0;
  char my[50];
  for (i=0;i<10;i++) {
    sprintf(my,"%d,%.2f",i,you);
    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)my,strlen(my));
    HAL_Delay(100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
