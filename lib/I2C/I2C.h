#include <Arduino.h>
#include <Wire.h>

void I2Cinit(void);
void I2CWriteReg(int8_t chipadr, int8_t regadr, int8_t data);
void I2CReadReg(int8_t chipadr, int8_t regadr, byte *temp);