#include "spl06.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "usart.h"

// 私有函数声明
static HAL_StatusTypeDef SPL06_ReadRegister(SPL06_HandleTypeDef *spl, uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef SPL06_WriteRegister(SPL06_HandleTypeDef *spl, uint8_t reg, uint8_t data);
static int32_t twos_complement(uint32_t raw, uint8_t bits);
static HAL_StatusTypeDef SPL06_ReadCalibration(SPL06_HandleTypeDef *spl);
static HAL_StatusTypeDef SPL06_ReadRawPressure(SPL06_HandleTypeDef *spl);
static HAL_StatusTypeDef SPL06_ReadRawTemperature(SPL06_HandleTypeDef *spl);
static void SPL06_CompensatePressure(SPL06_HandleTypeDef *spl);

/**
  * @brief  初始化SPL06-001气压传感器
  * @param  spl: 传感器句柄指针
  * @param  hi2c: I2C句柄指针
  * @param  address: I2C设备地址(SPL06_I2C_ADDR_DEFAULT或SPL06_I2C_ADDR_ALT)
  * @retval HAL状态
  */
HAL_StatusTypeDef SPL06_Init(SPL06_HandleTypeDef *spl, I2C_HandleTypeDef *hi2c, uint8_t address) {
    // 初始化传感器结构体
    spl->i2c_handle = hi2c;
    spl->i2c_addr = address;

    // 执行软复位
    HAL_StatusTypeDef status = SPL06_SoftReset(spl);
    if (status != HAL_OK) {
        return status;
    }

    // 等待传感器初始化完成(根据数据手册建议至少等待12ms)
    HAL_Delay(50);

    // 读取校准系数
    status = SPL06_ReadCalibration(spl);
    if (status != HAL_OK) {
        return status;
    }

    // 配置气压测量参数(64倍过采样)
    status = SPL06_WriteRegister(spl, SPL06_REG_PRS_CFG, SPL06_OVERSAMPLING_64);
    if (status != HAL_OK) {
        return status;
    }

    // 配置温度测量参数(64倍过采样，使用外部传感器)
    status = SPL06_WriteRegister(spl, SPL06_REG_TMP_CFG, (1 << 7) | SPL06_OVERSAMPLING_64);
    HAL_GPIO_WritePin(ld2_GPIO_Port,ld2_Pin,GPIO_PIN_SET);
    return status;
}

/**
  * @brief  执行单次气压测量
  * @param  spl: 传感器句柄指针
  * @retval HAL状态
  */
HAL_StatusTypeDef SPL06_MeasurePressureOnce(SPL06_HandleTypeDef *spl) {
    // 启动单次气压测量
    HAL_StatusTypeDef status = SPL06_WriteRegister(spl, SPL06_REG_MEAS_CFG, SPL06_MEAS_PRESSURE);
    if (status != HAL_OK) {
        return status;
    }

    // 等待测量完成(根据数据手册，64倍过采样约需要104ms)
    HAL_Delay(110);

    // 读取原始气压值
    status = SPL06_ReadRawPressure(spl);
    if (status != HAL_OK) {
        return status;
    }

    // 读取原始温度值(用于气压补偿)
    status = SPL06_ReadRawTemperature(spl);
    if (status != HAL_OK) {
        return status;
    }

    // 计算补偿后的气压值
    SPL06_CompensatePressure(spl);

    return HAL_OK;
}

/**
  * @brief  执行软复位
  * @param  spl: 传感器句柄指针
  * @retval HAL状态
  */
HAL_StatusTypeDef SPL06_SoftReset(SPL06_HandleTypeDef *spl) {
    // 写入0x09到复位寄存器执行软复位
    return SPL06_WriteRegister(spl, SPL06_REG_RESET, 0x09);
}

// 以下是私有函数实现 ----------------------------------------

/**
  * @brief  读取校准系数
  * @param  spl: 传感器句柄指针
  * @retval HAL状态
  */
static HAL_StatusTypeDef SPL06_ReadCalibration(SPL06_HandleTypeDef *spl) {
    uint8_t buffer[18] = {0};
    char temp[1000];
    // 一次性读取所有校准系数(18字节)
    HAL_StatusTypeDef status = SPL06_ReadRegister(spl, SPL06_REG_COEF, buffer, 18);
    if (status != HAL_OK) {
        return status;
    }
    for (int i = 0; i < 18; i++) {
        sprintf(temp,"第%d:%d\r\n",i,buffer[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)temp, strlen(temp), HAL_MAX_DELAY);
        //HAL_UART_Transmit_DMA(&huart1,temp,strlen(temp));
        HAL_Delay(100);
    }
    // 解析校准系数(转换为有符号数)
    spl->c0 = (buffer[0] << 4) | ((buffer[1] >> 4) & 0x0F);

    spl->c0 = twos_complement(spl->c0, 12);

    spl->c1 = ((buffer[1] & 0x0F) << 8) | buffer[2];
    spl->c1 = twos_complement(spl->c1, 12);

    spl->c00 = (buffer[3] << 12) | (buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);
    spl->c00 = twos_complement(spl->c00, 20);

    spl->c10 = ((buffer[5] & 0x0F) << 16) | (buffer[6] << 8) | buffer[7];
    spl->c10 = twos_complement(spl->c10, 20);

    spl->c01 = (buffer[8] << 8) | buffer[9];
    spl->c01 = twos_complement(spl->c01, 16);

    spl->c11 = (buffer[10] << 8) | buffer[11];
    spl->c11 = twos_complement(spl->c11, 16);

    spl->c20 = (buffer[12] << 8) | buffer[13];
    spl->c20 = twos_complement(spl->c20, 16);

    spl->c21 = (buffer[14] << 8) | buffer[15];
    spl->c21 = twos_complement(spl->c21, 16);

    spl->c30 = (buffer[16] << 8) | buffer[17];
    spl->c30 = twos_complement(spl->c30, 16);
    return HAL_OK;
}

/**
  * @brief  读取原始气压值
  * @param  spl: 传感器句柄指针
  * @retval HAL状态
  */
static HAL_StatusTypeDef SPL06_ReadRawPressure(SPL06_HandleTypeDef *spl) {
    uint8_t buffer[3];
    HAL_StatusTypeDef status = SPL06_ReadRegister(spl, SPL06_REG_PRS_B2, buffer, 3);
    if (status != HAL_OK) {
        return status;
    }

    // 合并3字节数据并转换为有符号数
    spl->raw_pressure = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    spl->raw_pressure = twos_complement(spl->raw_pressure, 24);

    return HAL_OK;
}

/**
  * @brief  读取原始温度值
  * @param  spl: 传感器句柄指针
  * @retval HAL状态
  */
static HAL_StatusTypeDef SPL06_ReadRawTemperature(SPL06_HandleTypeDef *spl) {
    uint8_t buffer[3];
    HAL_StatusTypeDef status = SPL06_ReadRegister(spl, SPL06_REG_TMP_B2, buffer, 3);
    if (status != HAL_OK) {
        return status;
    }

    // 合并3字节数据并转换为有符号数
    spl->raw_temperature = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    spl->raw_temperature = twos_complement(spl->raw_temperature, 24);

    return HAL_OK;
}

/**
  * @brief  计算补偿后的气压值
  * @param  spl: 传感器句柄指针
  */
static void SPL06_CompensatePressure(SPL06_HandleTypeDef *spl) {
    // 64倍过采样的比例因子
    const float kP = 1040384.0f;

    // 计算缩放后的原始值
    float p_raw_sc = (float)spl->raw_pressure / kP;
    float t_raw_sc = (float)spl->raw_temperature / kP;

    // 应用补偿公式(来自数据手册)
    spl->pressure = spl->c00 + p_raw_sc * (spl->c10 + p_raw_sc * (spl->c20 + p_raw_sc * spl->c30)) +
                    t_raw_sc * spl->c01 + t_raw_sc * p_raw_sc * (spl->c11 + p_raw_sc * spl->c21);
}

/**
  * @brief  从传感器读取寄存器值
  * @param  spl: 传感器句柄指针
  * @param  reg: 寄存器地址
  * @param  data: 数据缓冲区
  * @param  len: 要读取的数据长度
  * @retval HAL状态
  */
static HAL_StatusTypeDef SPL06_ReadRegister(SPL06_HandleTypeDef *spl, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(spl->i2c_handle, spl->i2c_addr << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  向传感器写入寄存器值
  * @param  spl: 传感器句柄指针
  * @param  reg: 寄存器地址
  * @param  data: 要写入的数据
  * @retval HAL状态
  */
static HAL_StatusTypeDef SPL06_WriteRegister(SPL06_HandleTypeDef *spl, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(spl->i2c_handle, spl->i2c_addr << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  将原始数据转换为有符号数(二进制补码)
  * @param  raw: 原始数据
  * @param  bits: 数据位数
  * @retval 转换后的有符号数
  */
static int32_t twos_complement(uint32_t raw, uint8_t bits) {
    if (raw & (1 << (bits - 1))) {
        // 负数处理
        return (int32_t)(raw - (1 << bits));
    }
    return (int32_t)raw;
}