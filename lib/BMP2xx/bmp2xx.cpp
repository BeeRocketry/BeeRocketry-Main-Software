#include "bmp2xx.h"

void bmpInit(void){
    setCtrlReg(0b001, 0b011, 0b11);
    setConfig(0b010, 0b000, 0b0);
}

void setCtrlReg(byte oversamplingTemp, byte oversamplingPressure, byte powerMode){
    byte temp = 0;
    temp = (oversamplingTemp << 5) | (oversamplingPressure << 2) | powerMode;
    I2CWriteReg(CHIP_ADR, REG_CTRL_MEAS, temp);
}

void setConfig(byte tStandby, byte filterSet, byte spi3w){
    byte temp = 0;
    temp = (tStandby << 5) | (filterSet << 2) | spi3w;
    I2CWriteReg(CHIP_ADR, REG_CONFIG, temp);
}

