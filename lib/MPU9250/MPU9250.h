#include <Arduino.h>
#include "I2C.h"

#define MPU_CHIPADR 0x00
#define MAG_CHIPADR 0x0C

#define WHOAMI 0x75

#define CONFIG 0x1A
#define CONFIG_GYRO 0x1B
#define CONFIG_ACCEL 0x1C
#define CONFIG_ACCEL_2 0x1D
#define POWER_MANAGEMENT 0x6B
#define POWER_MANAGEMENT 0x6C
#define USER_CTRL 0x6A
#define MAG_STATUS 0x2H
#define MAG_STATUS_2 0x9H
#define MAG_CTRL 0xAH
#define SAMPLERATE_DIVIDER 0x19
#define INT_PIN_CFG 0x37

#define GYRO_X_OFFSET_MSB 0x13
#define GYRO_X_OFFSET_LSB 0x14
#define GYRO_Y_OFFSET_MSB 0x15
#define GYRO_Y_OFFSET_LSB 0x16
#define GYRO_Z_OFFSET_MSB 0x17
#define GYRO_Z_OFFSET_LSB 0x18

#define ACCEL_X_OFFSET_MSB 0x77
#define ACCEL_X_OFFSET_LSB 0x78
#define ACCEL_Y_OFFSET_MSB 0x7A
#define ACCEL_Y_OFFSET_LSB 0x7B
#define ACCEL_Z_OFFSET_MSB 0x7D
#define ACCEL_Z_OFFSET_LSB 0x7E

#define ACCEL_X_OUTPUT_MSB 0x3B
#define ACCEL_X_OUTPUT_LSB 0x3C
#define ACCEL_Y_OUTPUT_MSB 0x3D
#define ACCEL_Y_OUTPUT_LSB 0x3E
#define ACCEL_Z_OUTPUT_MSB 0x3F
#define ACCEL_Z_OUTPUT_LSB 0x40

#define TEMP_OUTPUT_MSB 0x41
#define TEMP_OUTPUT_LSB 0x42

#define GYRO_X_OUTPUT_MSB 0x43
#define GYRO_X_OUTPUT_LSB 0x44
#define GYRO_Y_OUTPUT_MSB 0x45
#define GYRO_Y_OUTPUT_LSB 0x46
#define GYRO_Z_OUTPUT_MSB 0x47
#define GYRO_Z_OUTPUT_LSB 0x48

#define MAG_X_OUTPUT_LSB 0x03
#define MAG_X_OUTPUT_MSB 0x04
#define MAG_Y_OUTPUT_LSB 0x05
#define MAG_Y_OUTPUT_MSB 0x06
#define MAG_Z_OUTPUT_LSB 0x07
#define MAG_Z_OUTPUT_MSB 0x08

void mpuInit(void);
void setGyroRange(uint8_t range);
void setAccelRange(uint8_t range);
void setSleep(bool cond);
void setClock(uint8_t bits);