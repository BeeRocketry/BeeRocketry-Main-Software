#include "I2C.h"
#include <Arduino.h>

// Setting and Status Registers
#define REG_RESET 0xE0
#define REG_STATUS 0xF3
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define CHIP_ADR 0x77

// Data Registers
#define REG_PRESS_MSB 0xF7
#define REG_PRESS_LSB 0xF8
#define REG_PRESS_XLSB 0xF9
#define REG_TEMP_MSB 0xFA
#define REG_TEMP_LSB 0xFB
#define REG_TEMP_XLSB 0xFC

// Temperature Registers
#define REG_T1_LSB 0x88 // unsigned short
#define REG_T1_MSB 0x89
#define REG_T2_LSB 0x8A // signed short
#define REG_T2_MSB 0x8B
#define REG_T3_LSB 0x8C // signed short
#define REG_T3_MSB 0x8D

// Pressure Registers
#define REG_P1_LSB 0x8E // unsigned short
#define REG_P1_MSB 0x8F
#define REG_P2_LSB 0x90 // signed short
#define REG_P2_MSB 0x91
#define REG_P3_LSB 0x92 // signed short
#define REG_P3_MSB 0x93
#define REG_P4_LSB 0x94 // signed short
#define REG_P4_MSB 0x95
#define REG_P5_LSB 0x96 // signed short
#define REG_P5_MSB 0x97
#define REG_P6_LSB 0x98 // signed short
#define REG_P6_MSB 0x99
#define REG_P7_LSB 0x9A // signed short
#define REG_P7_MSB 0x9B
#define REG_P8_LSB 0x9C // signed short
#define REG_P8_MSB 0x9D
#define REG_P9_LSB 0x9E // signed short
#define REG_P9_MSB 0x9F
