#ifndef I2C_H
#define I2C_H

#define STM_I2C2_FREQ 400000
#define DATA_TO_SEND_LENGHT 30
#define TIMEOUT_I2C 1000

#include <Arduino.h>
#include <Wire.h>

void I2CBegin(uint32_t SDA, uint32_t SCL);
bool I2CWriteByte(uint8_t chipadr, uint8_t regadr, uint8_t data);
uint8_t I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint16_t timeout);
uint8_t I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint8_t length, uint16_t timeout);
uint8_t minimum(uint8_t x, uint8_t y);

#endif