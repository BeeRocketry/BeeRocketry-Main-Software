#include <Arduino.h>

#include "debugprinter.h"
#include "I2C.h"

void I2Cinit(uint32_t SDA, uint32_t SCL)
{
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();
    Wire.setClock(400000);
    DEBUG_PRINTLN(F("I2C Port Aktifleştirildi."));
}

bool I2CWriteByte(uint8_t chipadr, uint8_t regadr, uint8_t data)
{
    int8_t result;

    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.write(data);
    result = Wire.endTransmission(true);
    return result == 0;
}

uint8_t I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint16_t timeout)
{
    return I2CReadBytes(chipadr, regadr, temp, 1, timeout);
}

uint8_t I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint8_t length, uint16_t timeout)
{
    uint8_t cnt = 0;
    uint32_t t1 = millis();

    for(uint8_t check = 0; check < length; check += minimum(length, BUFFER_LENGTH)){
        Wire.beginTransmission(chipadr);
        Wire.write(regadr + check);
        Wire.endTransmission(false);

        Wire.requestFrom(chipadr, minimum(length - check, BUFFER_LENGTH));

        uint32_t start = millis();
        while(Wire.available()){
            if(timeout == 0 || millis() - start > timeout){
                return -1;
            }

            temp[cnt++] = Wire.read();
        }
    }

    return cnt;
}

uint8_t minimum(uint8_t x, uint8_t y)
{
    return (x < y) ? x : y;
}