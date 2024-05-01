#include "MPU9250.h"

void mpuInit(void){
    setClock(0b11);
    setGyroRange(0b01);
    setAccelRange(0b11);
    setSleep(false);
}

void setGyroRange(uint8_t range){
    uint8_t b;
    I2CReadByte(chipadr, CONFIG_GYRO, &b, TIMEOUT_I2C);
    b = b | (range << 3);
    I2CWriteByte(chipadr, CONFIG_GYRO, b);
}

void setAccelRange(uint8_t range){
    uint8_t b;
    I2CReadByte(chipadr, CONFIG_ACCEL, &b, TIMEOUT_I2C);
    b = b | (range << 3);
    I2CWriteByte(chipadr, CONFIG_ACCEL, b);
}

void setSleep(bool cond){
    uint8_t b;
    I2CReadByte(chipadr, POWER_MANAGEMENT, &b, TIMEOUT_I2C);
    b = b | (cond << 6);
    I2CWriteByte(chipadr, POWER_MANAGEMENT, b);
}

void setClock(uint8_t bits){
    uint8_t b;
    I2CReadByte(chipadr, POWER_MANAGEMENT, &b, TIMEOUT_I2C);
    b = b | bits;
    I2CWriteByte(chipadr, POWER_MANAGEMENT, b);
}