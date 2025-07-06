#include "AD9834.h"
#include "main.h"

static SPI_HandleTypeDef *AD9834_hspi;
static GPIO_TypeDef *AD9834_CS_Port;
static uint16_t AD9834_CS_Pin;
static uint32_t AD9834_MCLK = 75000000; // 默认75MHz时钟

/* 私有函数声明 */
static void AD9834_WriteRegister(uint16_t data);
static uint16_t AD9834_ReadRegister(void);

/**
  * @brief 初始化AD9834
  * @param hspi: SPI句柄指针
  * @param cs_port: 片选GPIO端口
  * @param cs_pin: 片选GPIO引脚
  * @param mclk_freq: AD9834的主时钟频率(Hz)
  * @retval 无
  */
void AD9834_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t mclk_freq)
{
    AD9834_hspi = hspi;
    AD9834_CS_Port = cs_port;
    AD9834_CS_Pin = cs_pin;
    AD9834_MCLK = mclk_freq;

    // 初始化CS引脚为高电平
    HAL_GPIO_WritePin(AD9834_CS_Port, AD9834_CS_Pin, GPIO_PIN_SET);

    // 复位AD9834
    AD9834_Reset();

    // 默认配置: 正弦波输出，使能28位装载
    AD9834_WriteRegister(AD9834_REG_CMD | AD9834_B28);

    // 设置默认频率和相位
    AD9834_SetFrequency(0, 1000000);  // 1MHz
    AD9834_SetPhase(0, 0);            // 0度相位
}

/**
  * @brief 复位AD9834
  * @retval 无
  */
void AD9834_Reset(void)
{
    AD9834_WriteRegister(AD9834_REG_CMD | AD9834_RESET);
    HAL_Delay(10); // 短暂延时确保复位完成
    AD9834_WriteRegister(AD9834_REG_CMD); // 清除复位位
}

/**
  * @brief 设置输出频率
  * @param freqReg: 频率寄存器选择(0=FREQ0, 1=FREQ1)
  * @param frequency: 输出频率(Hz)
  * @retval 无
  */
void AD9834_SetFrequency(uint8_t freqReg, uint32_t frequency)
{
    // 计算频率调谐字(FTW): FTW = (frequency * 2^28) / MCLK
    uint64_t ftw = ((uint64_t)frequency << 28) / AD9834_MCLK;

    if(freqReg == 0) {
        // 写入FREQ0寄存器(28位需要分两次写入)
        AD9834_WriteRegister(AD9834_REG_FREQ0 | AD9834_B28 | (ftw & 0x3FFF));
        AD9834_WriteRegister(AD9834_REG_FREQ0 | AD9834_B28 | ((ftw >> 14) & 0x3FFF));
    } else {
        // 写入FREQ1寄存器
        AD9834_WriteRegister(AD9834_REG_FREQ1 | AD9834_B28 | (ftw & 0x3FFF));
        AD9834_WriteRegister(AD9834_REG_FREQ1 | AD9834_B28 | ((ftw >> 14) & 0x3FFF));
    }
}

/**
  * @brief 设置输出相位
  * @param phaseReg: 相位寄存器选择(0=PHASE0, 1=PHASE1)
  * @param phase: 相位角度(0-360度)
  * @retval 无
  */
void AD9834_SetPhase(uint8_t phaseReg, uint16_t phase)
{
    // 相位值(0-4095对应0-2π)
    // 使用整数运算提高效率: phaseWord = (phase * 4096 + 180) / 360
    uint16_t phaseWord = (uint16_t)(((uint32_t)phase * 4096 + 180) / 360) & 0x0FFF;

    if(phaseReg == 0) {
        AD9834_WriteRegister(AD9834_REG_PHASE0 | phaseWord);
    } else {
        AD9834_WriteRegister(AD9834_REG_PHASE1 | phaseWord);
    }
}

/**
  * @brief 设置输出波形类型
  * @param waveform: 波形类型
  * @retval 无
  */
void AD9834_SetWaveform(AD9834_WaveformType waveform)
{
    // 读取当前控制寄存器配置(保留B28设置)
    uint16_t config = AD9834_ReadRegister() & AD9834_B28;

    // 添加波形配置
    config |= waveform;

    // 更新控制寄存器
    AD9834_WriteRegister(AD9834_REG_CMD | config);
}

/**
  * @brief 启用/禁用输出
  * @param enable: 1=启用, 0=禁用
  * @retval 无
  */
void AD9834_EnableOutput(uint8_t enable)
{
    uint16_t config = AD9834_ReadRegister();

    if(enable) {
        config &= ~AD9834_SLEEP1;  // 唤醒DAC
        config |= AD9834_OPBITEN;  // 使能输出
    } else {
        config |= AD9834_SLEEP1;   // 关闭DAC
        config &= ~AD9834_OPBITEN; // 禁用输出
    }

    AD9834_WriteRegister(AD9834_REG_CMD | config);
}

/**
  * @brief 设置睡眠模式
  * @param mode: 睡眠模式
  * @retval 无
  */
void AD9834_SetSleepMode(AD9834_SleepMode mode)
{
    uint16_t config = AD9834_ReadRegister();

    config &= ~(AD9834_SLEEP1 | AD9834_SLEEP12); // 清除睡眠位
    config |= mode; // 设置新的睡眠模式

    AD9834_WriteRegister(AD9834_REG_CMD | config);
}

/**
  * @brief 启用/禁用比较器输出
  * @param enable: 1=启用, 0=禁用
  * @retval 无
  */
void AD9834_EnableComparatorOutput(uint8_t enable)
{
    uint16_t config = AD9834_ReadRegister();

    if(enable) {
        config |= AD9834_OPBITEN | AD9834_SIGN_PIB;
    } else {
        config &= ~(AD9834_OPBITEN | AD9834_SIGN_PIB);
    }

    AD9834_WriteRegister(AD9834_REG_CMD | config);
}

/**
  * @brief 启用/禁用MSB输出
  * @param enable: 1=启用, 0=禁用
  * @param div2: 1=输出MSB, 0=输出MSB/2
  * @retval 无
  */
void AD9834_EnableMSBOutput(uint8_t enable, uint8_t div2)
{
    uint16_t config = AD9834_ReadRegister();

    if(enable) {
        config |= AD9834_OPBITEN;
        config &= ~AD9834_SIGN_PIB;
        if(div2) {
            config |= AD9834_DIV2;
        } else {
            config &= ~AD9834_DIV2;
        }
    } else {
        config &= ~AD9834_OPBITEN;
    }

    AD9834_WriteRegister(AD9834_REG_CMD | config);
}

/* 私有函数实现 */

/**
  * @brief 写入16位数据到AD9834
  * @param data: 要写入的16位数据
  * @retval 无
  */
static void AD9834_WriteRegister(uint16_t data)
{
    // 拉低CS(FSYNC)
    HAL_GPIO_WritePin(AD9834_CS_Port, AD9834_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // 确保>5ns
    // 发送16位数据(高位在前)
    uint8_t txData[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    HAL_SPI_Transmit(AD9834_hspi, txData, 2, HAL_MAX_DELAY);
    HAL_Delay(1); // 确保>5ns
    // 释放CS
    HAL_GPIO_WritePin(AD9834_CS_Port, AD9834_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief 读取当前控制寄存器值
  * @retval 当前控制寄存器值
  */
static uint16_t AD9834_ReadRegister(void)
{
    // AD9834没有直接读取寄存器的方法，需要跟踪最后写入的值
    static uint16_t lastControlReg = 0;
    return lastControlReg;
}