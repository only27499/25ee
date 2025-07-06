#include "AHT20.h"

#include <string.h>
#include "stdio.h"
#include "usart.h"

#define AHT20_ADDRESS 0x70


int AHT20_init(void) {
    uint8_t readBuffer;
    HAL_Delay(40);
    HAL_I2C_Master_Receive(&hi2c2,AHT20_ADDRESS,&readBuffer,1,HAL_MAX_DELAY);
    if ((readBuffer & 0x08) == 0x00) {
        uint8_t sendBuffer[3]={0xBE,0x08,0x00};
        HAL_I2C_Master_Transmit(&hi2c2,AHT20_ADDRESS,sendBuffer,3,HAL_MAX_DELAY);

    }
    HAL_GPIO_WritePin(ld2_GPIO_Port,ld2_Pin,GPIO_PIN_SET);
    return 0;
}

void AHT20_read(float *Temperature,float *Humidity) {
    uint8_t sendBuffer[3]={0xAC,0x33,0x00};
    uint8_t readBuffer[6];
    char sb[50];
    HAL_I2C_Master_Transmit(&hi2c2,AHT20_ADDRESS,sendBuffer,3,HAL_MAX_DELAY);
    HAL_Delay(75);
    HAL_I2C_Master_Receive(&hi2c2,AHT20_ADDRESS,readBuffer,6,HAL_MAX_DELAY);


    if ((readBuffer[0] & 0x80) == 0x00) {
        uint32_t data1=0,data2=0;
        char message[30];
        data1=((uint32_t)readBuffer[3]>>4) + ((uint32_t)readBuffer[2]<<4)+((uint32_t)readBuffer[1]<<12);
        *Humidity=(data1 *100.0f)/(1<<20);

        data2=(((uint32_t)readBuffer[3]&0x0F)<<16)+((uint32_t)readBuffer[4]<<8)+(uint32_t)readBuffer[5];
        *Temperature=(data2 *200.0f)/(1<<20)-50;

    }
}
void AHT20_SoftReset(void) {
    uint8_t cmd = 0xBA;
    HAL_I2C_Master_Transmit(&hi2c2, AHT20_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(20);  // 手册要求复位时间<20ms
}