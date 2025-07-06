/**
  * @file    ad7705.c
  * @brief   AD7705 ADC驱动源文件
  */

#include "ad7705.h"
#include "main.h"
#include <string.h>

// 私有函数声明
static void AD7705_CS_Low(AD7705_HandleTypeDef *hadc);
static void AD7705_CS_High(AD7705_HandleTypeDef *hadc);
static void AD7705_Delay(uint32_t delay);

/**
  * @brief  初始化AD7705
  * @param  hadc: AD7705句柄指针
  */
void AD7705_Init(AD7705_HandleTypeDef *hadc) {
    // 硬件复位
    AD7705_Reset(hadc);

    // 等待复位完成
    AD7705_Delay(10);

    // 设置时钟寄存器 (主时钟2.4576MHz，输出速率50Hz)
    AD7705_WriteRegister(hadc, AD7705_REG_CLOCK, 0x0C);

    // 初始通道配置 (AIN1, 增益1, 双极性, 50Hz)
    AD7705_SetupChannel(hadc, AD7705_CH_AIN1, AD7705_GAIN_1, AD7705_BIPOLAR, AD7705_FILTER_50HZ);

    // 执行自校准
    AD7705_Calibrate(hadc, AD7705_CH_AIN1, AD7705_MODE_SELF_CAL);

    // 配置第二个通道 (AIN2, 增益1, 双极性, 50Hz)
    AD7705_SetupChannel(hadc, AD7705_CH_AIN2, AD7705_GAIN_1, AD7705_BIPOLAR, AD7705_FILTER_50HZ);

    // 执行自校准
    AD7705_Calibrate(hadc, AD7705_CH_AIN2, AD7705_MODE_SELF_CAL);
}

/**
  * @brief  硬件复位AD7705
  * @param  hadc: AD7705句柄指针
  */
void AD7705_Reset(AD7705_HandleTypeDef *hadc) {
    HAL_GPIO_WritePin(hadc->reset_port, hadc->reset_pin, GPIO_PIN_RESET);
    AD7705_Delay(1);
    HAL_GPIO_WritePin(hadc->reset_port, hadc->reset_pin, GPIO_PIN_SET);
}

/**
  * @brief  写寄存器
  * @param  hadc: AD7705句柄指针
  * @param  reg: 寄存器地址
  * @param  data: 要写入的数据
  */
void AD7705_WriteRegister(AD7705_HandleTypeDef *hadc, uint8_t reg, uint8_t data) {
    AD7705_CS_Low(hadc);

    // 先写入通信寄存器配置
    uint8_t txData[2];
    txData[0] = reg;  // 寄存器地址
    txData[1] = data; // 寄存器数据

    HAL_SPI_Transmit(hadc->hspi, txData, 2, HAL_MAX_DELAY);

    AD7705_CS_High(hadc);
}

/**
  * @brief  读寄存器
  * @param  hadc: AD7705句柄指针
  * @param  reg: 寄存器地址
  * @retval 读取到的寄存器值
  */
uint8_t AD7705_ReadRegister(AD7705_HandleTypeDef *hadc, uint8_t reg) {
    AD7705_CS_Low(hadc);

    // 先写入通信寄存器配置
    uint8_t txData = reg | 0x08; // 设置读操作
    uint8_t rxData;

    HAL_SPI_Transmit(hadc->hspi, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hadc->hspi, &rxData, 1, HAL_MAX_DELAY);

    AD7705_CS_High(hadc);

    return rxData;
}

/**
  * @brief  读取ADC数据
  * @param  hadc: AD7705句柄指针
  * @param  channel: 通道选择
  * @retval 16位ADC值
  */
uint16_t AD7705_ReadData(AD7705_HandleTypeDef *hadc, uint8_t channel) {
    // 等待数据就绪
    while(!AD7705_DataReady(hadc, channel));

    AD7705_CS_Low(hadc);

    // 设置读取数据寄存器
    uint8_t txData = AD7705_REG_DATA | 0x08;
    uint8_t rxData[2];

    HAL_SPI_Transmit(hadc->hspi, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hadc->hspi, rxData, 2, HAL_MAX_DELAY);

    AD7705_CS_High(hadc);

    return (rxData[0] << 8) | rxData[1];
}

/**
  * @brief  配置ADC通道
  * @param  hadc: AD7705句柄指针
  * @param  channel: 通道选择
  * @param  gain: 增益设置
  * @param  polarity: 输入极性
  * @param  filter: 滤波器设置
  */
void AD7705_SetupChannel(AD7705_HandleTypeDef *hadc, uint8_t channel,
                         AD7705_Gain gain, AD7705_Polarity polarity,
                         AD7705_Filter filter) {
    // 配置通信寄存器选择通道
    uint8_t commReg = (channel << 1) & 0x03;
    AD7705_WriteRegister(hadc, AD7705_REG_COMM, commReg);

    // 配置设置寄存器
    uint8_t setupReg = 0;
    setupReg |= (gain << 4) & 0x70;    // 增益设置
    setupReg |= (polarity << 3) & 0x08; // 极性设置
    setupReg |= 0x00;                  // 缓冲关闭
    AD7705_WriteRegister(hadc, AD7705_REG_SETUP, setupReg);

    // 配置时钟寄存器
    uint8_t clockReg = 0;
    clockReg |= (filter << 2) & 0x0C;  // 滤波器设置
    clockReg |= 0x04;                  // 主时钟2.4576MHz
    AD7705_WriteRegister(hadc, AD7705_REG_CLOCK, clockReg);
}

/**
  * @brief  执行校准
  * @param  hadc: AD7705句柄指针
  * @param  channel: 通道选择
  * @param  mode: 校准模式
  */
void AD7705_Calibrate(AD7705_HandleTypeDef *hadc, uint8_t channel, AD7705_Mode mode) {
    // 选择通道
    uint8_t commReg = (channel << 1) & 0x03;
    AD7705_WriteRegister(hadc, AD7705_REG_COMM, commReg);

    // 配置校准模式
    uint8_t setupReg = 0;
    setupReg |= (mode << 6) & 0xC0;
    AD7705_WriteRegister(hadc, AD7705_REG_SETUP, setupReg);

    // 等待校准完成
    while(!AD7705_DataReady(hadc, channel));
}

/**
  * @brief  将原始ADC值转换为电压
  * @param  hadc: AD7705句柄指针
  * @param  raw: 原始ADC值
  * @param  gain: 增益设置
  * @param  polarity: 输入极性
  * @retval 计算得到的电压值(V)
  */
float AD7705_ConvertToVoltage(AD7705_HandleTypeDef *hadc, uint16_t raw,
                             AD7705_Gain gain, AD7705_Polarity polarity) {
    float gainValue = 1 << gain;

    if (polarity == AD7705_UNIPOLAR) {
        // 单极性模式: 0-Vref/gain
        return (raw / 65535.0f) * (hadc->vref / gainValue);
    } else {
        // 双极性模式: -Vref/gain ~ +Vref/gain
        return ((int16_t)raw / 32767.5f) * (hadc->vref / gainValue);
    }
}

/**
  * @brief  检查数据是否就绪
  * @param  hadc: AD7705句柄指针
  * @param  channel: 通道选择
  * @retval 1: 数据就绪, 0: 数据未就绪
  */
uint8_t AD7705_DataReady(AD7705_HandleTypeDef *hadc, uint8_t channel) {
    // 读取通信寄存器状态
    uint8_t commReg = (channel << 1) & 0x03;
    AD7705_WriteRegister(hadc, AD7705_REG_COMM, commReg | 0x08);

    uint8_t status = AD7705_ReadRegister(hadc, AD7705_REG_COMM);

    return !(status & 0x80); // DRDY位为0表示数据就绪
}

// 私有函数实现
static void AD7705_CS_Low(AD7705_HandleTypeDef *hadc) {
    HAL_GPIO_WritePin(hadc->cs_port, hadc->cs_pin, GPIO_PIN_RESET);
}

static void AD7705_CS_High(AD7705_HandleTypeDef *hadc) {
    HAL_GPIO_WritePin(hadc->cs_port, hadc->cs_pin, GPIO_PIN_SET);
}

static void AD7705_Delay(uint32_t delay) {
    HAL_Delay(delay);
}