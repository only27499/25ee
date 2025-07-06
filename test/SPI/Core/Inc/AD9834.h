#ifndef __AD9834_H__
#define __AD9834_H__
#ifdef __cplusplus
 extern "C" {
#endif

//#include "stm32g4xx_hal.h"
#include "spi.h"
//#include "main.h"
//extern SPI_HandleTypeDef hspi2;
     /* 寄存器地址定义 */
#define AD9834_REG_CMD        0x2000  // 控制寄存器
#define AD9834_REG_FREQ0      0x4000  // FREQ0寄存器
#define AD9834_REG_FREQ1      0x8000  // FREQ1寄存器
#define AD9834_REG_PHASE0     0xC000  // PHASE0寄存器
#define AD9834_REG_PHASE1     0xE000  // PHASE1寄存器

     /* 控制位定义 */
#define AD9834_B28           0x2000  // 28位装载使能
#define AD9834_HLB           0x1000  // 高位/低位字节选择
#define AD9834_FSEL          0x0800  // 频率寄存器选择
#define AD9834_PSEL          0x0400  // 相位寄存器选择
#define AD9834_RESET         0x0100  // 内部复位
#define AD9834_SLEEP1        0x0080  // DAC断电
#define AD9834_SLEEP12       0x0040  // 内部时钟禁用
#define AD9834_OPBITEN       0x0020  // 输出使能
#define AD9834_SIGN_PIB      0x0010  // SIGN PIB输出
#define AD9834_DIV2          0x0008  // 时钟除以2
#define AD9834_MODE          0x0002  // 模式选择(0=正弦,1=三角)

     /* 波形类型枚举 */
     typedef enum {
         AD9834_WAVE_SINE = 0,
         AD9834_WAVE_TRIANGLE = AD9834_MODE,
         AD9834_WAVE_SQUARE = AD9834_OPBITEN | AD9834_SIGN_PIB
     } AD9834_WaveformType;

     /* 睡眠模式枚举 */
     typedef enum {
         AD9834_SLEEP_NONE = 0,
         AD9834_SLEEP_DAC = AD9834_SLEEP12,
         AD9834_SLEEP_CLK = AD9834_SLEEP1,
         AD9834_SLEEP_ALL = AD9834_SLEEP1 | AD9834_SLEEP12
     } AD9834_SleepMode;

     /* 函数声明 */
     void AD9834_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t mclk_freq);
     void AD9834_Reset(void);
     void AD9834_SetFrequency(uint8_t freqReg, uint32_t frequency);
     void AD9834_SetPhase(uint8_t phaseReg, uint16_t phase);
     void AD9834_SetWaveform(AD9834_WaveformType waveform);
     void AD9834_EnableOutput(uint8_t enable);
     void AD9834_SetSleepMode(AD9834_SleepMode mode);
     void AD9834_EnableComparatorOutput(uint8_t enable);
     void AD9834_EnableMSBOutput(uint8_t enable, uint8_t div2);

#ifdef __cplusplus
 }
#endif

#endif /* __AD9834_H__ */