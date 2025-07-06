#include "bh1745.h"
#include <string.h>

/**
 * @brief 向BH1745写入一个字节数据
 * @param hbh1745: BH1745句柄
 * @param reg: 寄存器地址
 * @param value: 要写入的数据
 * @retval HAL状态
 */
static HAL_StatusTypeDef BH1745_WriteByte(BH1745_HandleTypeDef *hbh1745, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hbh1745->hi2c, hbh1745->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/**
 * @brief 从BH1745读取一个字节数据
 * @param hbh1745: BH1745句柄
 * @param reg: 寄存器地址
 * @param value: 读取的数据指针
 * @retval HAL状态
 */
static HAL_StatusTypeDef BH1745_ReadByte(BH1745_HandleTypeDef *hbh1745, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hbh1745->hi2c, hbh1745->address, reg, I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY);
}

/**
 * @brief 初始化BH1745
 * @param hbh1745: BH1745句柄
 * @param hi2c: I2C句柄
 * @param address: I2C地址
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_Init(BH1745_HandleTypeDef *hbh1745, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    uint8_t id;

    if(hbh1745 == NULL || hi2c == NULL)
        return HAL_ERROR;

    memset(hbh1745, 0, sizeof(BH1745_HandleTypeDef));
    hbh1745->hi2c = hi2c;
    hbh1745->address = address;
    hbh1745->gain = BH1745_GAIN_1X;
    hbh1745->measurement_time = BH1745_MEASUREMENT_TIME_160MS;

    // 检查设备ID
    if(BH1745_ReadByte(hbh1745, BH1745_MANUFACTURER_ID, &id) != HAL_OK)
        return HAL_ERROR;

    if(id != 0xE0) // BH1745制造商ID为0xE0
        return HAL_ERROR;

    // 复位设备
    if(BH1745_Reset(hbh1745) != HAL_OK)
        return HAL_ERROR;

    // 设置默认增益和测量时间
    if(BH1745_SetGain(hbh1745, hbh1745->gain) != HAL_OK)
        return HAL_ERROR;


    if(BH1745_SetMeasurementTime(hbh1745, hbh1745->measurement_time) != HAL_OK)
        return HAL_ERROR;
    // 启动测量
    return BH1745_StartMeasurement(hbh1745);
}



/**
 * @brief 复位BH1745
 * @param hbh1745: BH1745句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_Reset(BH1745_HandleTypeDef *hbh1745)
{
    // 写入0x07到系统控制寄存器进行软复位
    HAL_StatusTypeDef ret=BH1745_WriteByte(hbh1745, BH1745_SYSTEM_CONTROL, 0x07);
    HAL_Delay(2);
    return ret;
}

/**
 * @brief 设置BH1745增益
 * @param hbh1745: BH1745句柄
 * @param gain: 增益值
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_SetGain(BH1745_HandleTypeDef *hbh1745, BH1745_Gain gain)
{
    uint8_t reg_value;
    if(BH1745_ReadByte(hbh1745, BH1745_MODE_CONTROL2, &reg_value) != HAL_OK)
        return HAL_ERROR;
    reg_value &= ~0x03; // 清除增益位
    reg_value |= (gain & 0x03);
    hbh1745->gain = gain;
    return BH1745_WriteByte(hbh1745, BH1745_MODE_CONTROL2, reg_value);
}

/**
 * @brief 设置BH1745测量时间
 * @param hbh1745: BH1745句柄
 * @param time: 测量时间
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_SetMeasurementTime(BH1745_HandleTypeDef *hbh1745, BH1745_MeasurementTime time)
{
    uint8_t reg_value;
    if(BH1745_ReadByte(hbh1745, BH1745_MODE_CONTROL1, &reg_value) != HAL_OK)
        return HAL_ERROR;
    reg_value &= ~0x07; // 清除测量时间位
    reg_value |= (time & 0x07);
    hbh1745->measurement_time = time;
    return BH1745_WriteByte(hbh1745, BH1745_MODE_CONTROL1, reg_value);
}
/**
 * @brief 启动BH1745测量
 * @param hbh1745: BH1745句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_StartMeasurement(BH1745_HandleTypeDef *hbh1745)
{
    uint8_t reg_value;
    if(BH1745_ReadByte(hbh1745, BH1745_MODE_CONTROL2, &reg_value) != HAL_OK)
        return HAL_ERROR;
    reg_value |= 0x10; // 设置RGBC测量使能位
    return BH1745_WriteByte(hbh1745, BH1745_MODE_CONTROL2, reg_value);
}

/**
 * @brief 停止BH1745测量
 * @param hbh1745: BH1745句柄
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_StopMeasurement(BH1745_HandleTypeDef *hbh1745)
{
    uint8_t reg_value;
    if(BH1745_ReadByte(hbh1745, BH1745_MODE_CONTROL2, &reg_value) != HAL_OK)
        return HAL_ERROR;
    reg_value &= ~0x10; // 清除RGBC测量使能位
    return BH1745_WriteByte(hbh1745, BH1745_MODE_CONTROL2, reg_value);
}

/**
 * @brief 读取BH1745 RGB数据
 * @param hbh1745: BH1745句柄
 * @param rgb_data: RGB数据指针
 * @retval HAL状态
 */
HAL_StatusTypeDef BH1745_ReadRGB(BH1745_HandleTypeDef *hbh1745, BH1745_RGBData *rgb_data)
{   uint8_t status;
    if (BH1745_ReadByte(hbh1745, BH1745_MODE_CONTROL2, &status) != HAL_OK)
        return HAL_ERROR;
    if (!(status & 0x80)) // 检查VALID位（bit7）
        return HAL_ERROR; // 数据未就绪
    uint8_t data[8];
    if(HAL_I2C_Mem_Read(hbh1745->hi2c, hbh1745->address, BH1745_RED_DATA_LSB,
                        I2C_MEMADD_SIZE_8BIT, data, 8, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;
    rgb_data->red = (data[1] << 8) | data[0];
    rgb_data->green = (data[3] << 8) | data[2];
    rgb_data->blue = (data[5] << 8) | data[4];
    rgb_data->clear = (data[7] << 8) | data[6];
    return HAL_OK;
}
/**
 * @brief 读取BH1745制造商ID
 * @param hbh1745: BH1745句柄
 * @retval 制造商ID
 */
uint8_t BH1745_ReadManufacturerID(BH1745_HandleTypeDef *hbh1745)
{
    uint8_t id;
    if(BH1745_ReadByte(hbh1745, BH1745_MANUFACTURER_ID, &id) != HAL_OK)
        return 0;
    return id;
}