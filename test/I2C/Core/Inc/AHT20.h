#ifndef AHT20_H
#define AHT20_H

#include "i2c.h"

int AHT20_init(void);
void AHT20_read(float *Temperature,float *Humidity);
void AHT20_SoftReset(void);
#endif