#ifndef BME280__H
#define BME280__H

#include <Arduino.h>
#include "I2C.h"

#define SeaLevel 1013.25f

#define BME_I2C_ADR 0x77

#define BME_REG_CHIP_ADR 0xD0
#define BME_REG_CONFIG 0xF5
#define BME_REG_CTRL_MEAS 0xF4
#define BME_REG_CTRL_HUM 0xF2
#define BME_REG_HUM_LSB 0xFE
#define BME_REG_HUM_MSB 0xFD
#define BME_REG_TEMP_XLSB 0xFC
#define BME_REG_TEMP_LSB 0xFB
#define BME_REG_TEMP_MSB 0xFA
#define BME_REG_PRESS_XLSB 0xF9
#define BME_REG_PRESS_LSB 0xF8
#define BME_REG_PRESS_MSB 0xF7
#define BME_REG_STATUS 0xF3
#define BME_REG_RESET 0xE0

#define BME_REG_T1_LSB 0x88 // unsigned short
#define BME_REG_T1_MSB 0x89
#define BME_REG_T2_LSB 0x8A // signed short
#define BME_REG_T2_MSB 0x8B
#define BME_REG_T3_LSB 0x8C // signed short
#define BME_REG_T3_MSB 0x8D

#define BME_REG_P1_LSB 0x8E // unsigned short
#define BME_REG_P1_MSB 0x8F
#define BME_REG_P2_LSB 0x90 // signed short
#define BME_REG_P2_MSB 0x91
#define BME_REG_P3_LSB 0x92 // signed short
#define BME_REG_P3_MSB 0x93
#define BME_REG_P4_LSB 0x94 // signed short
#define BME_REG_P4_MSB 0x95
#define BME_REG_P5_LSB 0x96 // signed short
#define BME_REG_P5_MSB 0x97
#define BME_REG_P6_LSB 0x98 // signed short
#define BME_REG_P6_MSB 0x99
#define BME_REG_P7_LSB 0x9A // signed short
#define BME_REG_P7_MSB 0x9B
#define BME_REG_P8_LSB 0x9C // signed short
#define BME_REG_P8_MSB 0x9D
#define BME_REG_P9_LSB 0x9E // signed short
#define BME_REG_P9_MSB 0x9F

#define BME_REG_H1 0xA1 // unsigned char
#define BME_REG_H2_LSB 0xE1 // signed short
#define BME_REG_H2_MSB 0xE2
#define BME_REG_H3 0xE3 // unsigned char
#define BME_REG_H4_LSB 0xE5 // signed short
#define BME_REG_H4_MSB 0xE4
#define BME_REG_H5_LSB 0xE5 // signed short
#define BME_REG_H5_MSB 0xE6

typedef enum : uint8_t{
    BME_OVERSAMPLING_OFF = 0b000,
    BME_OVERSAMPLING_1X = 0b001,
    BME_OVERSAMPLING_2X = 0b010,
    BME_OVERSAMPLING_4X = 0b011,
    BME_OVERSAMPLING_8X = 0b100,
    BME_OVERSAMPLING_16X = 0b101
}BME_OVERSAMPLING;

typedef enum : uint8_t{
    BME_POWERMODE_Sleep = 0b00,
    BME_POWERMODE_Forced = 0b01,
    BME_POWERMODE_Normal = 0b11
}BME_POWERMODE;

typedef enum : uint8_t{
    BME_IIR_OFF = 0b000,
    BME_IIR_2X = 0b001,
    BME_IIR_4X = 0b010,
    BME_IIR_8X = 0b011,
    BME_IIR_16X = 0b100,
}BME_IIR_COEFFICIENT;

typedef enum : uint8_t{
    BME_STANDBY_0a5 = 0b000,
    BME_STANDBY_62a5 = 0b001,
    BME_STANDBY_125 = 0b010,
    BME_STANDBY_250 = 0b011,
    BME_STANDBY_500 = 0b100,
    BME_STANDBY_1000 = 0b101,
    BME_STANDBY_10 = 0b110,
    BME_STANDBY_20 = 0b111
}BME_STANDBY;

typedef enum : uint8_t{
    BME_GENERAL_ON = 0b1,
    BME_GENERAL_OFF = 0b0
}BME_GENERALONOFF;

struct BME_STR_CONFIG{
    BME_STANDBY standbyTime = BME_STANDBY_10; // 7-5 Bit
    BME_IIR_COEFFICIENT IIRConfig = BME_IIR_OFF; // 4-2 Bit
    BME_GENERALONOFF spi3wire = BME_GENERAL_OFF; // 0. Bit
};

struct BME_STR_CTRL_MEAS{
    BME_OVERSAMPLING tempSampling = BME_OVERSAMPLING_4X; // 7-5 Bit
    BME_OVERSAMPLING pressSampling = BME_OVERSAMPLING_8X; // 4-2 Bit
    BME_POWERMODE powerMode = BME_POWERMODE_Normal; // 1-0 Bit
};

struct BME_STR_CTRL_HUM{
    BME_OVERSAMPLING humSampling = BME_OVERSAMPLING_8X; // 2-0 Bit
};

struct BME_STR_REGISTERS{
    BME_STR_CONFIG configReg;
    BME_STR_CTRL_MEAS controlMeas;
    BME_STR_CTRL_HUM controlHum;
};

struct BME_CalibData{
    uint16_t T1;
    int16_t T2;
    int16_t T3;

    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;

    uint8_t H1;
    int16_t H2;
    uint8_t H3;
    int16_t H4;
    int16_t H5;
    int8_t H6;
};

struct BME_DATA{
    float temp;
    float press;
    float hum;
    float altitude;
    int32_t tfine;
    int32_t tfineAdjust = 0;
};

void BMEGetData(BME_DATA *data);
void readAltitude(BME_DATA *data, float sealevel);
void readHumidity(BME_DATA *data);
uint8_t readPressure(BME_DATA *data);
void readTemperature(BME_DATA *data);
void setControlHumidRegister(BME_OVERSAMPLING humSamp);
void setConfigRegister(BME_STANDBY standby, BME_IIR_COEFFICIENT iirFilter, BME_GENERALONOFF spiwire);
void setSleep();
void setControlMeasRegister(BME_OVERSAMPLING tempSamp, BME_OVERSAMPLING pressSamp, BME_POWERMODE powerMode);
void setConfig(BME_STR_REGISTERS ayarlar);
void readTrimValues(BME_CalibData *calibrationValues);
bool isCalibrating();
void BMEBegin(BME_OVERSAMPLING basincOversampling, BME_OVERSAMPLING sicaklikOversampling, BME_OVERSAMPLING nemOversampling, BME_IIR_COEFFICIENT iirFilter);
void BMEReset();

#endif