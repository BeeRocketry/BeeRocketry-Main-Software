#include <Arduino.h>
#include <Wire.h>

#define TIMEOUT_I2C 1000

void I2Cinit(void);
void I2CWriteReg(int8_t chipadr, int8_t regadr, int8_t data);
int8_t I2CReadReg(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint16_t timeout);
int8_t I2CReadRegMulti(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint8_t length, uint16_t timeout);