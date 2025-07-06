#ifndef SPL06_H
#define SPL06_H

#include "stm32g4xx_hal.h"

// 传感器I2C地址定义
#define SPL06_I2C_ADDR_DEFAULT 0x77  // 默认I2C地址
#define SPL06_I2C_ADDR_ALT     0x76  // 备用I2C地址（当SDO引脚接地时）

// 寄存器地址定义
#define SPL06_REG_PRS_B2        0x00  // 气压数据高位寄存器
#define SPL06_REG_PRS_B1        0x01  // 气压数据中位寄存器
#define SPL06_REG_PRS_B0        0x02  // 气压数据低位寄存器
#define SPL06_REG_TMP_B2        0x03  // 温度数据高位寄存器
#define SPL06_REG_TMP_B1        0x04  // 温度数据中位寄存器
#define SPL06_REG_TMP_B0        0x05  // 温度数据低位寄存器
#define SPL06_REG_PRS_CFG       0x06  // 气压配置寄存器
#define SPL06_REG_TMP_CFG       0x07  // 温度配置寄存器
#define SPL06_REG_MEAS_CFG      0x08  // 测量配置寄存器
#define SPL06_REG_CFG_REG       0x09  // 配置寄存器
#define SPL06_REG_RESET         0x0C  // 复位寄存器
#define SPL06_REG_COEF          0x10  // 校准系数起始寄存器

// 测量模式定义
#define SPL06_MEAS_IDLE         0x00  // 空闲模式
#define SPL06_MEAS_PRESSURE     0x01  // 单次气压测量
#define SPL06_MEAS_TEMPERATURE  0x02  // 单次温度测量

// 过采样率定义
#define SPL06_OVERSAMPLING_1    0x00  // 1倍过采样
#define SPL06_OVERSAMPLING_2    0x01  // 2倍过采样
#define SPL06_OVERSAMPLING_4    0x02  // 4倍过采样
#define SPL06_OVERSAMPLING_8    0x03  // 8倍过采样
#define SPL06_OVERSAMPLING_16   0x04  // 16倍过采样
#define SPL06_OVERSAMPLING_64   0x06  // 64倍过采样（高精度）

// 传感器数据结构体
typedef struct {
    I2C_HandleTypeDef *i2c_handle;  // I2C句柄指针
    uint8_t i2c_addr;               // I2C设备地址

    // 校准系数
    int16_t c0, c1;
    int32_t c00, c10, c20, c30;
    int16_t c01, c11, c21;

    // 原始测量值
    int32_t raw_pressure;     // 原始气压值
    int32_t raw_temperature;  // 原始温度值

    // 补偿后的值
    float pressure;           // 补偿后的气压值(Pa)
    float temperature;        // 补偿后的温度值(°C)
} SPL06_HandleTypeDef;

// 函数声明
HAL_StatusTypeDef SPL06_Init(SPL06_HandleTypeDef *spl, I2C_HandleTypeDef *hi2c, uint8_t address);
HAL_StatusTypeDef SPL06_MeasurePressureOnce(SPL06_HandleTypeDef *spl);
HAL_StatusTypeDef SPL06_SoftReset(SPL06_HandleTypeDef *spl);

#endif // SPL06_H