#include <Arduino.h>

#define STM_I2C2_FREQ 400000
#define DATA_TO_SEND_LENGHT 30

#include <Wire.h>
#include "I2C.h"

void I2Cinit(int32_t SDA, int32_t SCL)
{
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();
    Wire.setClock(400000);
    Serial2.println("I2C Port Aktifleştirildi.");
}

bool I2CWriteByte(int8_t chipadr, int8_t regadr, int8_t data)
{
    int8_t result;

    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.write(data);
    result = Wire.endTransmission();
    return result == 0;
}

int8_t I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint16_t timeout)
{
    int8_t cnt = 0;
    uint8_t length = 1;
    uint32_t t1 = millis();
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.beginTransmission(chipadr);
    Wire.requestFrom(chipadr, (uint8_t)1);

    while (Wire.available() && (timeout == 0 || millis() - t1 < timeout))
    {
        *temp = Wire.read();
        cnt++;
    }

    Wire.endTransmission();

    if (timeout > 0 && millis() - t1 >= timeout && cnt < length)
    {
        cnt = -1;
    }

    return cnt;
}

int8_t I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint8_t length, uint16_t timeout)
{
    int8_t cnt = 0;
    uint32_t t1 = millis();

    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.requestFrom(chipadr, length);

    while(Wire.available() && (timeout == 0 || millis() - t1 < timeout)){
        temp[cnt++] = Wire.read();
    }


    /*
    for(uint8_t check = 0; check < length; check += minimum(length, BUFFER_LENGTH)){
        Wire.beginTransmission(chipadr);
        Wire.write(regadr);
        Wire.endTransmission();

        Wire.beginTransmission(chipadr);
        Wire.requestFrom(chipadr, (uint8_t)(length - check, BUFFER_LENGTH));

        while(Wire.available() && (timeout == 0 || millis() - t1 < timeout)){
            temp[cnt++] = Wire.read();
        }

        Wire.endTransmission();
    }*/

    if (timeout > 0 && millis() - t1 >= timeout && cnt < length)
    {
        cnt = -1;
    }

    return cnt;
}

int8_t minimum(uint8_t x, uint8_t y)
{
    if (x > y)
    {
        return y;
    }
    return x;
}