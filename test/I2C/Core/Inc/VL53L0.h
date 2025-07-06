#ifndef _VL53L0X_H_
#define _VL53L0X_H_

#include "main.h"
#include <stdbool.h>



#define VL53L0X_DEFAULT_I2C_ADDR1 0x29  ///< The fixed I2C addres

#define SYSRANGE_START                              0x00

#define SYSTEM_THRESH_HIGH                          0x0C
#define SYSTEM_THRESH_LOW                           0x0E

#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_RANGE_CONFIG                         0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04

#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A

#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define SYSTEM_INTERRUPT_CLEAR                      0x0B

#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14

#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF                 0xB6

#define ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28

#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A

#define MSRC_CONFIG_CONTROL                         0x60

#define PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64

#define FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62

#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52

#define SYSTEM_HISTOGRAM_BIN                        0x81
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define HISTOGRAM_CONFIG_READOUT_CTRL               0x55

#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46

#define SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define IDENTIFICATION_MODEL_ID                     0xC0
#define IDENTIFICATION_REVISION_ID                  0xC2

#define OSC_CALIBRATE_VAL                           0xF8

#define GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5

#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE            0x80

#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

#define ALGO_PHASECAL_LIM                           0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT                0x30




//写入1个字节数据
void VL53L0X_WriteByte(uint8_t add,uint8_t reg,uint8_t data);
//写入2个字节数据
void VL53L0X_WriteByte_16Bit(uint8_t add,uint8_t reg,uint16_t data);
//写入4个字节数据
void VL53L0X_WriteByte_32Bit(uint8_t add,uint8_t reg,uint32_t data);

//读取1个字节数据
uint8_t VL53L0X_ReadByte(uint8_t add,uint8_t reg);
//读取2个字节数据
uint16_t VL53L0X_ReadBytee_16Bit(uint8_t add,uint16_t reg);
//从传感器的指定寄存器开始，读取任意数量的字节，并存放到给定的数组中。
void VL53L0X_readMulti(uint8_t add,uint8_t reg,  uint8_t * dst, uint8_t count);
//从给定的数组中写入任意数量的字节到传感器，从指定的寄存器开始。
void VL53L0X_writeMulti(uint8_t add,uint8_t reg, uint8_t * src, uint8_t count);


//初始化程序
uint8_t VL53L0X_Init(uint8_t add,bool io_2v8);

//VL53L0X时域飞行（Time-of-Flight，ToF）传感器设置返回信号的速率限制
uint8_t setSignalRateLimit(uint8_t add,float limit_Mcps);

//设定传感器进行一次测量所需要的时间，称为“测量时间预算”（measurement timing budget）。
//这个预算时间被细分成多个子步骤，每个子步骤都需要一定的时间，该函数的目标是确保所有子步骤的总时间不超过设定的预算。
//较长的时间预算：可以得到更精确的测量结果。
bool VL53L0X_setMeasurementTimingBudget(uint8_t add,uint32_t budget_us);

//获取传感器当前设置的测量时间预算，单位是微秒（us）。
uint32_t VL53L0X_getMeasurementTimingBudget(uint8_t add);

//从VL53L0X Time-of-Flight (ToF) 距离传感器中获取VCSEL（垂直腔面发射激光器）的脉冲周期，单位是PCLKs。
//这是一种用于测量物体距离的激光源。
uint8_t VL53L0X_getVcselPulsePeriod(uint8_t add,uint8_t type);


//启动连续测距模式。
//如果period_ms是0或未给出，那么会使用连续back-to-back模式（传感器尽可能快地连续进行测量）。
//如果提供了period_ms，那么将使用连续定时模式，传感器会根据给定的间隔周期（毫秒）进行测量。
void VL53L0X_startContinuous(uint8_t add,uint32_t period_ms);



//从VL53L0X ToF距离传感器中获取关于SPAD（单光子雪崩二极管）的参考计数和类型信息。
uint16_t VL53L0X_readRangeSingleMillimeters(uint8_t add);

//通过与传感器的通信接口，特定地读取和写入到某些寄存器，来获取参考SPAD的信息。
bool VL53L0X_getSpadInfo(uint8_t add,uint8_t * count, bool * type_is_aperture);

//从VL53L0X ToF距离传感器的特定寄存器中读取的值解码为MCLKs（主时钟周期）中的超时。
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val);

//编码超时值以供存储在VL53L0X的寄存器中。
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);


//这个函数从给定的超时值（以MCLKs为单位）编码出一个可以存储在传感器寄存器中的16位值。
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val);
//写入VL53L0X传感器的相应寄存器以设置超时
uint16_t VL53L0X_encodeTimeout(uint32_t timeout_mclks);



//将一个超时值从主时钟周期（MCLKs）转换为微秒，同时考虑到一个给定的VCSEL（垂直腔表面发射激光器）周期（以PCLKs为单位）。
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

//这个函数从给定的超时值（以微秒为单位）和VCSEL周期（以PCLKs为单位）计算出超时值的MCLKs表示。
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

//这个函数执行一个单次的参考校准，这通常是在初始化传感器或改变某些配置后进行的。
//它通过给定的 vhv_init_byte 启动一个测量，并等待测量完成，然后清除中断并停止测量。
bool VL53L0X_performSingleRefCalibration(uint8_t add,uint8_t vhv_init_byte);





//该函数返回当传感器处于连续测量模式时的距离读数（以毫米为单位）。
uint16_t VL53L0X_readRangeContinuousMillimeters(uint8_t add);
//执行单次测量并返回测量的距离值（以毫米为单位）。
uint16_t VL53L0X_readRangeSingleMillimeters(uint8_t add);










#endif // _TOFLIB_H

