#include "MPU9250.h"

void mpuInit(void){
    setClock(0b11);
    setGyroRange(0b01);
    setAccelRange(0b11);
    setSleep(false);
}

void setGyroRange(uint8_t range){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, CONFIG_GYRO, &b, TIMEOUT_I2C);
    b = b | (range << 3);
    I2CWriteByte(MPU_CHIPADR, CONFIG_GYRO, b);
}

void setAccelRange(uint8_t range){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, CONFIG_ACCEL, &b, TIMEOUT_I2C);
    b = b | (range << 3);
    I2CWriteByte(MPU_CHIPADR, CONFIG_ACCEL, b);
}

void setSleep(bool cond){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, POWER_MANAGEMENT, &b, TIMEOUT_I2C);
    b = b | (cond << 6);
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, b);
}

void setClock(uint8_t bits){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, POWER_MANAGEMENT, &b, TIMEOUT_I2C);
    b = b | bits;
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, b);
}

void getRawAccel(int16_t *accel){
    int16_t accelx, accely, accelz;
    uint8_t buff[6];
    I2CReadBytes(MPU_CHIPADR, ACCEL_X_OUTPUT_MSB, buff, 6, TIMEOUT_I2C);

    accelx = (((int16_t)buff[0]) << 8) | buff[1];
    accely = (((int16_t)buff[2]) << 8) | buff[3];
    accelz = (((int16_t)buff[4]) << 8) | buff[5];

    accel[0] = accelx;
    accel[1] = accely;
    accel[2] = accelz;
}

void getRawGyro(int16_t *gyro){
    int16_t gyrox, gyroy, gyroz;
    uint8_t buff[6];

    I2CReadBytes(MPU_CHIPADR, GYRO_X_OUTPUT_MSB, buff, 6, TIMEOUT_I2C);

    gyrox = (((int16_t)buff[0]) << 8) | buff[1];
    gyroy = (((int16_t)buff[2]) << 8) | buff[3];
    gyroz = (((int16_t)buff[4]) << 8) | buff[5];

    gyro[0] = gyrox;
    gyro[1] = gyroy;
    gyro[2] = gyroz;
}

void getRawMag(int16_t *mag){
    uint8_t buff[6];
    int16_t magx, magy, magz;

    I2CWriteByte(MPU_CHIPADR, INT_PIN_CFG, 0x02);
    delay(10);
    I2CWriteByte(MAG_CHIPADR, 0x0A, 0x01);
    delay(10);
    I2CReadBytes(MAG_CHIPADR, MAG_X_OUTPUT_LSB, buff, 6, TIMEOUT_I2C);
    
    magx = (((int16_t)buff[1]) << 8) | buff[0];
    magy = (((int16_t)buff[3]) << 8) | buff[2];
    magz = (((int16_t)buff[5]) << 8) | buff[4];

    mag[0] = magx;
    mag[1] = magy;
    mag[2] = magz;
}