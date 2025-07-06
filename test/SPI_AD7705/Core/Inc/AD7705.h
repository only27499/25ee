/**
  * @file    ad7705.h
  * @brief   AD7705 ADC驱动头文件
  *
  * 功能：
  * - 支持双通道差分输入
  * - 可编程增益(1-128)
  * - 自动校准功能
  * - SPI接口通信
  */

#ifndef __AD7705_H
#define __AD7705_H

#include "stm32g4xx_hal.h"

// AD7705寄存器定义
#define AD7705_REG_COMM    0x00  // 通信寄存器
#define AD7705_REG_SETUP   0x10  // 设置寄存器
#define AD7705_REG_CLOCK   0x20  // 时钟寄存器
#define AD7705_REG_DATA    0x30  // 数据寄存器
#define AD7705_REG_TEST    0x40  // 测试寄存器
#define AD7705_REG_ZERO    0x60  // 零点校准寄存器
#define AD7705_REG_FULL    0x70  // 满量程校准寄存器

// 通道选择
#define AD7705_CH_AIN1     0x00  // AIN1+ / AIN1-
#define AD7705_CH_AIN2     0x01  // AIN2+ / AIN2-
#define AD7705_CH_SELF_TEST 0x02 // 自测试模式

// 增益设置
typedef enum {
    AD7705_GAIN_1 = 0,
    AD7705_GAIN_2,
    AD7705_GAIN_4,
    AD7705_GAIN_8,
    AD7705_GAIN_16,
    AD7705_GAIN_32,
    AD7705_GAIN_64,
    AD7705_GAIN_128
} AD7705_Gain;

// 工作模式
typedef enum {
    AD7705_MODE_NORMAL = 0,
    AD7705_MODE_SELF_CAL,
    AD7705_MODE_ZERO_CAL,
    AD7705_MODE_FULL_CAL
} AD7705_Mode;

// 输入极性
typedef enum {
    AD7705_BIPOLAR = 0,
    AD7705_UNIPOLAR
} AD7705_Polarity;

// 滤波器设置
typedef enum {
    AD7705_FILTER_50HZ = 0,
    AD7705_FILTER_60HZ,
    AD7705_FILTER_250HZ,
    AD7705_FILTER_500HZ
} AD7705_Filter;

// AD7705配置结构体
typedef struct {
    SPI_HandleTypeDef *hspi;     // SPI句柄
    GPIO_TypeDef *cs_port;       // CS端口
    uint16_t cs_pin;            // CS引脚
    GPIO_TypeDef *drdy_port;     // DRDY端口
    uint16_t drdy_pin;          // DRDY引脚
    GPIO_TypeDef *reset_port;    // RESET端口
    uint16_t reset_pin;         // RESET引脚
    float vref;                 // 参考电压(V)
} AD7705_HandleTypeDef;

// 函数声明
void AD7705_Init(AD7705_HandleTypeDef *hadc);
void AD7705_Reset(AD7705_HandleTypeDef *hadc);
void AD7705_WriteRegister(AD7705_HandleTypeDef *hadc, uint8_t reg, uint8_t data);
uint8_t AD7705_ReadRegister(AD7705_HandleTypeDef *hadc, uint8_t reg);
uint16_t AD7705_ReadData(AD7705_HandleTypeDef *hadc, uint8_t channel);
void AD7705_SetupChannel(AD7705_HandleTypeDef *hadc, uint8_t channel, AD7705_Gain gain,
                         AD7705_Polarity polarity, AD7705_Filter filter);
void AD7705_Calibrate(AD7705_HandleTypeDef *hadc, uint8_t channel, AD7705_Mode mode);
float AD7705_ConvertToVoltage(AD7705_HandleTypeDef *hadc, uint16_t raw, AD7705_Gain gain, AD7705_Polarity polarity);
uint8_t AD7705_DataReady(AD7705_HandleTypeDef *hadc, uint8_t channel);

#endif /* __AD7705_H */