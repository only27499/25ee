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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "AHT20.h"
#include "SPL06.h"
#include "SPL06_001.h"
#include "BH1745.h"
#include "VL53L0.h"
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
char message[50];
char message1[50];
char message2[50];
char message3[50];
char message4[50];
char message5[50];
char receivedata[50];
char all[150];
SPL06_HandleTypeDef hspi06;  // 气压传感器实例
extern DMA_HandleTypeDef hdma_usart1_rx;//
BH1745_HandleTypeDef hbh1745;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart==&huart1) {
    HAL_UART_Transmit_DMA(huart, (uint8_t*)message, strlen(message));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)receivedata, sizeof(receivedata));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);//关闭过半中断
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart1,receivedata,sizeof(receivedata));
  //__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);//关闭过半中断



  /*-------------------------*/

  //         温湿度


  int a=AHT20_init();
  if (a!=0) {
    sprintf(message2,"AHT20 failed\r\n");
    HAL_UART_Transmit(&huart1,message2,strlen(message2),HAL_MAX_DELAY);
    //Error_Handler();
  }
  else {
    sprintf(message2,"AHT20 success\r\n");
    HAL_UART_Transmit(&huart1,message2,strlen(message2),HAL_MAX_DELAY);
  }
  HAL_Delay(10);
  /*-------------------------*/


  //  气压


  // if (SPL06_Init(&hspi06, &hi2c2, SPL06_I2C_ADDR_DEFAULT) != HAL_OK) {
  //   // 初始化失败处理
  //   Error_Handler();
  // }

  /*-----------------------------------------------------------------------------------------------*/

  //                 气压


  int assure=spl06_init();
  if(assure==0){
    sprintf(message1,"SP106__SUCCESS!!!\r\n");
    HAL_UART_Transmit(&huart1,(uint8_t*)message1,strlen(message1),500);
  }
  else {
    sprintf(message1,"SP106__failed!!!\r\n");
    HAL_UART_Transmit(&huart1,(uint8_t*)message1,strlen(message1),500);
    //Error_Handler();
  }
  HAL_Delay(10);
  /*--------------------------------------------------------------------------------------*/

  //                光


  // BH1745_RGBData rgb_data;
  // if (HAL_I2C_IsDeviceReady(&hi2c2, BH1745_ADDR_HIGH << 1, 3, 100) != HAL_OK)
  // {
  //   sprintf(message3, "Device not found!\r\n");
  //   HAL_UART_Transmit(&huart1, (uint8_t*)message3, strlen(message3), 500);
  // }
  // // 初始化BH1745
  // if(BH1745_Init(&hbh1745, &hi2c2, BH1745_ADDR_HIGH) != HAL_OK)
  // {
  //   sprintf(message3, "BH1745初始化失败\r\n");
  //   HAL_UART_Transmit(&huart1, (uint8_t*)message3, strlen(message3), 500);
  //   while (1)
  //   {}
  // }
  // sprintf(message3,"BH1745制造商ID: 0x%02X\r\n",BH1745_ReadManufacturerID(&hbh1745));
  // HAL_UART_Transmit(&huart1, (uint8_t*)message3, strlen(message3), 500);
  //
  // // 设置增益为16x
  // BH1745_SetGain(&hbh1745, BH1745_GAIN_16X);
  //
  // // 设置测量时间为640ms
  // BH1745_SetMeasurementTime(&hbh1745, BH1745_MEASUREMENT_TIME_640MS);

  /*------------------------------------------------------------------------*/
  /*------------------------------------------------------------------------*/
  if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1,true))
  {
    sprintf(message5,"VL53L0X failed\r\n");
    HAL_UART_Transmit(&huart1,(uint8_t*)message5,strlen(message5),HAL_MAX_DELAY);
    //Error_Handler();
  }
  else {
    sprintf(message5,"VL53L0X success\r\n");
    HAL_UART_Transmit(&huart1,(uint8_t*)message5,strlen(message5),HAL_MAX_DELAY);
  }
  HAL_Delay(10);
  /*------------------------------------------------------------------------*/
  float AHT_t,humidity,pressure,SPL_T,temperature;
  int i,L;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



    /*----------------测气压--------------*/
    // if (SPL06_MeasurePressureOnce(&hspi06) == HAL_OK) {
    //   // 测量成功，可以访问hspi06.pressure获取气压值(单位:Pa)
    //   pressure=hspi06.pressure;
    //   //sprintf(message1, "Pressure: %.2f Pa", pressure);
    //   //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message1, strlen(message1));
    //
    //   // 例如转换为hPa: float pressure_hPa = hspi06.pressure / 100.0f;
    // } else {
    //   // 测量失败处理
    //   HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
    // }
    /*------------------------------------*/

    /*--------------新的测气压-----------*/
    spl06_get_result(&pressure,&SPL_T);
    HAL_Delay(100);
    /*----------------------------------*/

    /*  -----测温湿度--------  */
    AHT20_read(&AHT_t,&humidity);
     // sprintf(message2,"Temperature: %.1f °C, Humidity: %.1f %%\r\n",AHT_t,humidity);
     // HAL_UART_Transmit_DMA(&huart1,(uint8_t*)message2,strlen(message2));
    HAL_Delay(100);
    /*------------------------*/

    /*---------------测光强色温----------------*/

    // 读取RGB数据
    // if(BH1745_ReadRGB(&hbh1745, &rgb_data) == HAL_OK)
    // {
    //   sprintf(message3, "R: %5d, G: %5d, B: %5d, C: %5d\r\n",rgb_data.red, rgb_data.green, rgb_data.blue, rgb_data.clear);
    //   HAL_UART_Transmit(&huart1, (uint8_t*)message3, strlen(message3), 100);
    // }
    // else
    // {
    //   sprintf(message3,"读取RGB数据失败\r\n");
    // }

    /*----------------------------------------*/



    L=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR1);
    // sprintf(message5,"%d",L);
    // HAL_UART_Transmit(&huart1, (uint8_t*)message5, strlen(message5), 500);
    HAL_Delay(100);

    temperature=(AHT_t+SPL_T)/2;
    sprintf(all,"Temprature:%.2f °C; Pressure:%.2f hpa; SPL_T:%.2f °C; AHT_t:%.2f °C; humidity:%.1f %%; DIS: %d mm",
      temperature,pressure,SPL_T,AHT_t,humidity,L);

    HAL_UART_Transmit(&huart1,(uint8_t*)all,strlen(all),500);
    HAL_Delay(1000);
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
