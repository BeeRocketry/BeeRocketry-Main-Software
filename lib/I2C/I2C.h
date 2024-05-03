#include <Arduino.h>
#include <Wire.h>

#define TIMEOUT_I2C 1000

void I2Cinit(int32_t SDA, int32_t SCL);
bool I2CWriteByte(int8_t chipadr, int8_t regadr, int8_t data);
int8_t I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint16_t timeout);
int8_t I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint8_t length, uint16_t timeout);