#ifndef BH1745_H
#define  bH1745_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32g4xx_hal.h"
#define BH1745_ADDR_LOW 0x38
#define BH1745_ADDR_HIGH 0x39

  /* BH1745寄存器地址 */
#define BH1745_SYSTEM_CONTROL 0x40
#define BH1745_MODE_CONTROL1 0x41
#define BH1745_MODE_CONTROL2 0x42
#define BH1745_MODE_CONTROL3 0x44
#define BH1745_RED_DATA_LSB 0x50
#define BH1745_RED_DATA_MSB 0x51
#define BH1745_GREEN_DATA_LSB 0x52
#define BH1745_GREEN_DATA_MSB 0x53
#define BH1745_BLUE_DATA_LSB 0x54
#define BH1745_BLUE_DATA_MSB 0x55
#define BH1745_CLEAR_DATA_LSB 0x56
#define BH1745_CLEAR_DATA_MSB 0x57
#define BH1745_DINT_DATA_LSB 0x58
#define BH1745_DINT_DATA_MSB 0x59
#define BH1745_INTERRUPT 0x60
#define BH1745_PERSISTENCE 0x61
#define BH1745_TH_LSB 0x62
#define BH1745_TH_MSB 0x63
#define BH1745_TL_LSB 0x64
#define BH1745_TL_MSB 0x65
#define BH1745_MANUFACTURER_ID 0x92

     /* 测量时间设置 */
     typedef enum {
         BH1745_MEASUREMENT_TIME_160MS = 0x00,
         BH1745_MEASUREMENT_TIME_320MS = 0x01,
         BH1745_MEASUREMENT_TIME_640MS = 0x02,
         BH1745_MEASUREMENT_TIME_1280MS = 0x03,
         BH1745_MEASUREMENT_TIME_2560MS = 0x04,
         BH1745_MEASUREMENT_TIME_5120MS = 0x05
     } BH1745_MeasurementTime;

     /* 增益设置 */
     typedef enum {
         BH1745_GAIN_1X = 0x00,
         BH1745_GAIN_2X = 0x01,
         BH1745_GAIN_16X = 0x02
     } BH1745_Gain;

     /* RGB数据 */
     typedef struct {
         uint16_t red;
         uint16_t green;
         uint16_t blue;
         uint16_t clear;
     } BH1745_RGBData;

     /* 设备句柄 */
     typedef struct {
         I2C_HandleTypeDef *hi2c;
         uint8_t address;
         BH1745_Gain gain;
         BH1745_MeasurementTime measurement_time;
     } BH1745_HandleTypeDef;

     /* 函数声明 */
     HAL_StatusTypeDef BH1745_Init(BH1745_HandleTypeDef *hbh1745, I2C_HandleTypeDef *hi2c, uint8_t address);
     HAL_StatusTypeDef BH1745_Reset(BH1745_HandleTypeDef *hbh1745);
     HAL_StatusTypeDef BH1745_SetGain(BH1745_HandleTypeDef *hbh1745, BH1745_Gain gain);
     HAL_StatusTypeDef BH1745_SetMeasurementTime(BH1745_HandleTypeDef *hbh1745, BH1745_MeasurementTime time);
     HAL_StatusTypeDef BH1745_StartMeasurement(BH1745_HandleTypeDef *hbh1745);
     HAL_StatusTypeDef BH1745_StopMeasurement(BH1745_HandleTypeDef *hbh1745);
     HAL_StatusTypeDef BH1745_ReadRGB(BH1745_HandleTypeDef *hbh1745, BH1745_RGBData *rgb_data);
     uint8_t BH1745_ReadManufacturerID(BH1745_HandleTypeDef *hbh1745);

#ifdef __cplusplus
 }
#endif



#endif

