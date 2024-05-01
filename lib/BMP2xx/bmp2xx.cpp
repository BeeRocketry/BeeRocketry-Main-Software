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

int32_t getRawTemp(void){
    int i = 0;
    int error = 5;
    uint32_t temporary[3] = {0};
    Wire.beginTransmission(CHIP_ADR);
    Wire.write(0xFA);
    error = Wire.endTransmission();
    Wire.requestFrom(CHIP_ADR, 3);

    while(Wire.available()){
        temporary[i++] = Wire.read();
    }

    int32_t temp = 0;
    temp = (temporary[2] << 12) | (temporary[1] << 3) | (temporary[2] >> 4);

    return temp;
}

int32_t getCompensatedTemp(int32_t rawData, int32_t *tfine){
    int32_t var1, var2, T;
    int32_t dig_T1 = 0, dig_T2 = 0, dig_T3 = 0;

    getTempCalb(&dig_T1, &dig_T2, &dig_T3);

    var1 = (((rawData >> 3) - (dig_T1 <<1)) * dig_T2) >> 11;
    var2 = (((((rawData >> 4) - dig_T1) * ((rawData >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;

    *tfine = var1 + var2;
    T = (*tfine * 5 + 128) >> 8;

    return T;
}

void getTempCalb(int32_t *T1, int32_t *T2, int32_t *T3){
    uint8_t buff[2];

    I2CReadReg(CHIP_ADR, REG_T1_LSB, buff, TIMEOUT_I2C);
    I2CReadReg(CHIP_ADR, REG_T1_MSB, &buff[1], TIMEOUT_I2C);
    *T1 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_T2_LSB, buff, TIMEOUT_I2C);
    I2CReadReg(CHIP_ADR, REG_T2_MSB, &buff[1], TIMEOUT_I2C);
    *T2 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_T3_LSB, buff, TIMEOUT_I2C);
    I2CReadReg(CHIP_ADR, REG_T3_MSB, &buff[1], TIMEOUT_I2C);
    *T3 = (buff[1] << 8) | buff[0];
}

int32_t getRawPres(void){
    uint8_t temporary[3];
    I2CReadReg(CHIP_ADR, REG_PRESS_LSB, &temporary[1], TIMEOUT_I2C);
    I2CReadReg(CHIP_ADR, REG_PRESS_MSB, &temporary[2], TIMEOUT_I2C);
    I2CReadReg(CHIP_ADR, REG_PRESS_XLSB, &temporary[0], TIMEOUT_I2C);

    int32_t press = 0;
    press = (temporary[2] << 12) | (temporary[1] << 3) | (temporary[2] >> 4);

    return press;
}

uint32_t getCompensatedPres(int32_t rawData, int32_t tfine){
    int32_t var1, var2;
    uint32_t P;
    int32_t dig_P1 = 0, dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;

    getPresCalb(&dig_P1, &dig_P2, &dig_P3, &dig_P4, &dig_P5, &dig_P6, &dig_P7, &dig_P8, &dig_P9);

    var1 = (tfine >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (dig_P6);
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * dig_P1) >> 15;

    if(var1 == 0){
        return 0;
    }

    P = (((uint32_t)(((int32_t)1048576) - rawData) - (var2 >> 12))) * 3125;

    if (P < 0x80000000)
    {
        P = (P << 1) / ((uint32_t)var1);
    }
    else{
        P = (P / (uint32_t)var1) * 2;
    }

    var1 = (dig_P9 * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(P >> 2)) * dig_P8) >> 13;

    P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

void getPresCalb(int32_t *P1, int32_t *P2, int32_t *P3, int32_t *P4, int32_t *P5, int32_t *P6, int32_t *P7, int32_t *P8, int32_t *P9){
    uint8_t buff[18];

    I2CReadRegMulti(CHIP_ADR, REG_P1_LSB, buff, 18, TIMEOUT_I2C);
   
    for(int i = 0; i < 9; i++){
        int32_t value = 0;
        value = (buff[i * 2 + 1] << 8) | buff[i * 2];
        switch (i)
        {
            case 0: *P1 = value; break;
            case 1: *P2 = value; break;
            case 2: *P3 = value; break;
            case 3: *P4 = value; break;
            case 4: *P5 = value; break; 
            case 5: *P6 = value; break;
            case 6: *P7 = value; break;
            case 7: *P8 = value; break;
            case 8: *P9 = value; break;
        }
    }
}

/* void getPresCalb(int32_t *P1, int32_t *P2, int32_t *P3, int32_t *P4, int32_t *P5, int32_t *P6, int32_t *P7, int32_t *P8, int32_t *P9){
    int32_t buff[2];
   
    I2CReadReg(CHIP_ADR, REG_P1_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P1_MSB, &buff[1]);
    *P1 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P2_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P2_MSB, &buff[1]);
    *P2 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P3_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P3_MSB, &buff[1]);
    *P3 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P1_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P1_MSB, &buff[1]);
    *P4 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P2_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P2_MSB, &buff[1]);
    *P5 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P3_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P3_MSB, &buff[1]);
    *P6 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P1_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P1_MSB, &buff[1]);
    *P7 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P2_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P2_MSB, &buff[1]);
    *P8 = (buff[1] << 8) | buff[0];

    I2CReadReg(CHIP_ADR, REG_P3_LSB, buff);
    I2CReadReg(CHIP_ADR, REG_P3_MSB, &buff[1]);
    *P9 = (buff[1] << 8) | buff[0];
} */