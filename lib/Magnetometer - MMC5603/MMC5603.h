#ifndef MMC5603_H
#define MMC5603_H

#include <Arduino.h>
#include "debugprinter.h"
#include "I2C.h"

#define MMC5603_CHIPADR 0x30

#define MMC_X_OUTPUT_MSB 0x00
#define MMC_X_OUTPUT_LSB 0x01
#define MMC_Y_OUTPUT_MSB 0x02
#define MMC_Y_OUTPUT_LSB 0x03
#define MMC_Z_OUTPUT_MSB 0x04
#define MMC_Z_OUTPUT_LSB 0x05

#define MMC_X_OUTPUT_Extra 0x06
#define MMC_Y_OUTPUT_Extra 0x07
#define MMC_Z_OUTPUT_Extra 0x08

#define MMC_Temp_OUTPUT 0x09

#define MMC_Status 0x18
#define MMC_ODR 0x1A
#define MMC_Control0 0x1B
#define MMC_Control1 0x1C
#define MMC_Control2 0x1D
#define MMC_ProductID 0x39

struct Dof3Data_IntMAG{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
};

struct Dof3Data_FloatMMC{
    float x = 0;
    float y = 0;
    float z = 0;
};

void MMCBegin(bool continuousmode, uint16_t datarate);
void getMagData(Dof3Data_IntMAG *data, Dof3Data_FloatMMC *magdata);

#endif