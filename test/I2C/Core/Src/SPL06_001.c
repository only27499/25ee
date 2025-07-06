#include "SPL06_001.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
static float _kT, _kP;
static int16_t _c0, _c1, _c01,_c11, _c20, _c21, _c30;
static int32_t _c10, _c00;
spl06_result_t spl06_result = {  
    .Praw = 0,  
    .Traw = 0,  
    .Pcomp = 0.0f,  
    .Tcomp = 0.0f  
};
static void _spl06_pressure_config(uint8_t rate, uint8_t oversampling)
{
    uint8_t regval = 0;

    switch(oversampling)
    {
        case PM_PRC_1:   _kP = 524288;  break;
        case PM_PRC_2:   _kP = 1572864; break;
        case PM_PRC_4:   _kP = 3670016; break;
        case PM_PRC_8:   _kP = 7864320; break;
        case PM_PRC_16:  _kP = 253952;  break;
        case PM_PRC_32:  _kP = 516096;  break;
        case PM_PRC_64:  _kP = 1040384; break;
        case PM_PRC_128: _kP = 2088960; break;
        default: break;
    }

    regval = rate | oversampling;
    //i2c_write(SPL06_ADDR, SP06_PSR_CFG, 1, &regval);
		HAL_I2C_Mem_Write(&hi2c2, SPL06_ADDR,SP06_PSR_CFG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);

    if(oversampling > PM_PRC_8)
    {
        //i2c_read(SPL06_ADDR, SP06_CFG_REG, 1, &regval);
			  HAL_I2C_Mem_Read(&hi2c2, SPL06_ADDR, SP06_CFG_REG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);
        regval |= SPL06_CFG_P_SHIFT;
        //i2c_write(SPL06_ADDR, SP06_CFG_REG, 1, &regval);
				HAL_I2C_Mem_Write(&hi2c2, SPL06_ADDR,SP06_CFG_REG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);
    }
}

static void _spl06_temperature_config(uint8_t rate, uint8_t oversampling)
{
    uint8_t regval = 0;
    
    switch(oversampling)
    {
        case TMP_PRC_1:   _kT = 524288;  break;
        case TMP_PRC_2:   _kT = 1572864; break;
        case TMP_PRC_4:   _kT = 3670016; break;
        case TMP_PRC_8:   _kT = 7864320; break;
        case TMP_PRC_16:  _kT = 253952;  break;
        case TMP_PRC_32:  _kT = 516096;  break;
        case TMP_PRC_64:  _kT = 1040384; break;
        case TMP_PRC_128: _kT = 2088960; break;
        default: break;
    }
    
    regval = rate | oversampling | 0x80;
    //i2c_write(SPL06_ADDR, SP06_TMP_CFG, 1, &regval);    /* External sensor */
    HAL_I2C_Mem_Write(&hi2c2, SPL06_ADDR,SP06_TMP_CFG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);
    if(oversampling > PM_PRC_8)
    {
        //i2c_read(SPL06_ADDR, SP06_CFG_REG, 1, &regval);
				HAL_I2C_Mem_Read(&hi2c2, SPL06_ADDR, SP06_CFG_REG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);
        regval |= SPL06_CFG_T_SHIFT;
        //i2c_write(SPL06_ADDR, SP06_CFG_REG, 1, &regval);
				HAL_I2C_Mem_Write(&hi2c2, SPL06_ADDR,SP06_CFG_REG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);
    }
}

int spl06_init(void)
{
    uint8_t regval = 0;
    uint8_t coef[18] = {0};
  /*
    //i2c_write(SPL06_ADDR, SP06_RESET, 1, &regval);
    HAL_I2C_Mem_Write(&hi2c1, SPL06_ADDR,SP06_RESET, I2C_MEMADD_SIZE_8BIT, &regval, 1, 5000);
    //i2c_read(SPL06_ADDR, SP06_ID, 1, &regval);
    HAL_I2C_Mem_Read(&hi2c1, SPL06_ADDR,SP06_ID, I2C_MEMADD_SIZE_8BIT, &regval, 1, 500);
		
		HAL_UART_Transmit(&hlpuart1, &regval, 10, 500); 
		
    if (0x10 != regval)
    {
        return -1;
    }
    
    HAL_Delay(50);
    */
 
		HAL_I2C_Mem_Read(&hi2c2, SPL06_ADDR,SP06_COEF, I2C_MEMADD_SIZE_8BIT, coef, 18, 500);

    _c0 = ((int16_t)coef[0] << 4) | ((coef[1] & 0xF0) >> 4);
    _c0 = (_c0 & 0x0800) ? (0xF000 | _c0) : _c0;
    _c1 = ((int16_t)(coef[1] & 0x0F) << 8) | coef[2];
    _c1 = (_c1 & 0x0800) ? (0xF000 | _c1) : _c1;
    _c00 = ((int32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4 ) | (coef[5] >> 4);
    _c00 = (_c00 & 0x080000) ? (0xFFF00000 | _c00) : _c00;
    _c10 = ((int32_t)(coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | coef[7];
    _c10 = (_c10 & 0x080000) ? (0xFFF00000 | _c10) : _c10;
    _c01 = ((int16_t)coef[8] << 8)  | coef[9];
    _c11 = ((int16_t)coef[10] << 8) | coef[11];
    _c20 = ((int16_t)coef[12] << 8) | coef[13];
    _c21 = ((int16_t)coef[14] << 8) | coef[15];
    _c30 = ((int16_t)coef[16] << 8) | coef[17];
    
    _spl06_pressure_config(PM_RATE_8, PM_PRC_64);
    _spl06_temperature_config(TMP_RATE_1, TMP_PRC_1);
    
    /* �������� */
    regval = MEAS_CTRL_ContinuousPressTemp;
    //i2c_write(SPL06_ADDR, SP06_MEAS_CFG, 1, &regval);
    HAL_I2C_Mem_Write(&hi2c2, SPL06_ADDR,SP06_MEAS_CFG, I2C_MEMADD_SIZE_8BIT, &regval, 1, 5000);

    return 0;
}

void spl06_get_raw_data()
{
    uint8_t regval[3] = {0};
    
    //i2c_read(SPL06_ADDR, SP06_PSR_B2, 3, regval);
		HAL_I2C_Mem_Read(&hi2c2, SPL06_ADDR,SP06_PSR_B2, I2C_MEMADD_SIZE_8BIT, regval, 3, 5000);
    spl06_result.Praw = (int32_t)regval[0] << 16 | (int32_t)regval[1] << 8 | regval[2];
    spl06_result.Praw = (spl06_result.Praw & 0x00800000) ? (0xFF000000 | spl06_result.Praw) :spl06_result.Praw;
    HAL_Delay(20);
    //i2c_read(SPL06_ADDR, SP06_TMP_B2, 3, regval);
		HAL_I2C_Mem_Read(&hi2c2, SPL06_ADDR,SP06_TMP_B2, I2C_MEMADD_SIZE_8BIT, regval, 3, 5000);
    spl06_result.Traw = (int32_t)regval[0] << 16 | (int32_t)regval[1] << 8 | regval[2];
    spl06_result.Traw = (spl06_result.Traw & 0x00800000) ? (0xFF000000 | spl06_result.Traw) : spl06_result.Traw;
}

void spl06_get_result(float* press,float* temp)
{
    spl06_get_raw_data();

    float Praw_sc = spl06_result.Praw / _kP;
    float Traw_sc = spl06_result.Traw / _kT;
		//spl06_result.Pcomp = 0;
	  //spl06_result.Tcomp = 0;
    spl06_result.Pcomp = _c00 
                 + Praw_sc * (_c10 + Praw_sc * (_c20 + Praw_sc * _c30))
                 + Traw_sc * _c01
                 + Traw_sc * Praw_sc * (_c11 + Praw_sc * _c21);
    
    spl06_result.Tcomp = _c0 * 0.5 + _c1 * Traw_sc;
    *press = spl06_result.Pcomp;
    *temp = spl06_result.Tcomp;
		// char buffer[100];
  //   sprintf(buffer, "Pressure: %.2f hPa, Temperature: %.2f C\r\n", spl06_result.Pcomp, spl06_result.Tcomp);
  //   HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 500);
}


