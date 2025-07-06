#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_def.h"

#include "sys.h"
#include "delay.h"
#include "usart.h"


//#define VL_XSHUT_RCC  RCC_APB2Periph_GPIOA
#define VL_XSHUT_PIN  GPIO_Pin_15
#define VL_XSHUT_IOx  GPIOA  


//控制Xshut电平,从而使能VL53L0X工作
#define XSHUT_HIGH   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET)
#define XSHUT_LOW    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET)




VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr);
VL53L0X_Error VL53L0x_Init(VL53L0X_Dev_t *pDev);
VL53L0X_Error VL53L0x_set_mode(VL53L0X_Dev_t *dev, uint8_t mode);
VL53L0X_Error VL53L1_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata);
void print_pal_error(VL53L0X_Error Status);
#endif


