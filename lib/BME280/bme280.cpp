#include "bme280.h"

BME_CalibData *calibrationValues;

void BMEReset(){
    I2CWriteByte(BME_I2C_ADR, BME_REG_RESET, 0xB6);
    delay(20);
};

void BMEBegin(BME_OVERSAMPLING basincOversampling, BME_OVERSAMPLING sicaklikOversampling, BME_OVERSAMPLING nemOversampling, BME_IIR_COEFFICIENT iirFilter){
    BME_STR_REGISTERS bmeRegister;

    bmeRegister.controlHum.humSampling = nemOversampling;
    bmeRegister.controlMeas.pressSampling = basincOversampling;
    bmeRegister.controlMeas.tempSampling = sicaklikOversampling;
    bmeRegister.configReg.IIRConfig = iirFilter;

    BMEReset();

    while(isCalibrating()){
        delay(10);
    }

    calibrationValues = (BME_CalibData*)malloc(sizeof(BME_CalibData));

    readTrimValues(calibrationValues);

    setConfig(bmeRegister);
}

bool isCalibrating(){
    uint8_t reg = 0;
    I2CReadByte(BME_I2C_ADR, BME_REG_STATUS, &reg, TIMEOUT_I2C);

    return (reg & (1 << 0)) != 0;
}

void readTrimValues(BME_CalibData *calibrationValues){
    uint8_t buffer[32];

    I2CReadBytes(BME_I2C_ADR, BME_REG_T1_LSB, buffer, 24, TIMEOUT_I2C);

    delay(5);

    I2CReadByte(BME_I2C_ADR, BME_REG_H1, &buffer[24], TIMEOUT_I2C);

    delay(5);

    I2CReadBytes(BME_I2C_ADR, BME_REG_H2_LSB, &buffer[25], 7, TIMEOUT_I2C);

    calibrationValues->T1 = (uint16_t)(((uint16_t)buffer[1] << 8) | (buffer[0]));
    calibrationValues->T2 = (int16_t)(((uint16_t)buffer[3] << 8) | (buffer[2]));
    calibrationValues->T3 = (int16_t)(((uint16_t)buffer[5] << 8) | (buffer[4]));

    calibrationValues->P1 = (uint16_t)(((uint16_t)buffer[7] << 8) | (buffer[6]));
    calibrationValues->P2 = (int16_t)(((uint16_t)buffer[9] << 8) | (buffer[8]));
    calibrationValues->P3 = (int16_t)(((uint16_t)buffer[11] << 8) | (buffer[10]));
    calibrationValues->P4 = (int16_t)(((uint16_t)buffer[13] << 8) | (buffer[12]));
    calibrationValues->P5 = (int16_t)(((uint16_t)buffer[15] << 8) | (buffer[14]));
    calibrationValues->P6 = (int16_t)(((uint16_t)buffer[17] << 8) | (buffer[16]));
    calibrationValues->P7 = (int16_t)(((uint16_t)buffer[19] << 8) | (buffer[18]));
    calibrationValues->P8 = (int16_t)(((uint16_t)buffer[21] << 8) | (buffer[20]));
    calibrationValues->P9 = (int16_t)(((uint16_t)buffer[23] << 8) | (buffer[22]));

    calibrationValues->H1 = (uint8_t)buffer[24];
    calibrationValues->H2 = (int16_t)(((uint16_t)buffer[26] << 8) | (buffer[25]));
    calibrationValues->H3 = (uint8_t)buffer[27];
    calibrationValues->H4 = (int16_t)(((uint16_t)buffer[28] << 4) | (buffer[29] & 0b00001111));
    calibrationValues->H5 = (int16_t)(((uint16_t)buffer[30] << 4) | (buffer[29] >> 4));
    calibrationValues->H3 = (int8_t)buffer[31];
}

void setConfig(BME_STR_REGISTERS ayarlar){
    setSleep();

    delay(10);

    setControlHumidRegister(ayarlar.controlHum.humSampling);
    setConfigRegister(ayarlar.configReg.standbyTime, ayarlar.configReg.IIRConfig, ayarlar.configReg.spi3wire);
    setControlMeasRegister(ayarlar.controlMeas.tempSampling, ayarlar.controlMeas.pressSampling, ayarlar.controlMeas.powerMode);
}

void setControlMeasRegister(BME_OVERSAMPLING tempSamp, BME_OVERSAMPLING pressSamp, BME_POWERMODE powerMode){
    uint8_t reg = 0;

    reg = ((uint8_t)tempSamp << 5) | ((uint8_t)pressSamp << 2) | (uint8_t)powerMode;

    I2CWriteByte(BME_I2C_ADR, BME_REG_CTRL_MEAS, reg);
}

void setSleep(){
    setControlMeasRegister(BME_OVERSAMPLING_4X, BME_OVERSAMPLING_8X, BME_POWERMODE_Sleep);
}

void setConfigRegister(BME_STANDBY standby, BME_IIR_COEFFICIENT iirFilter, BME_GENERALONOFF spiwire){
    uint8_t reg = 0;

    reg = ((uint8_t)standby << 5) | ((uint8_t)iirFilter << 2) | (uint8_t)spiwire;
    I2CWriteByte(BME_I2C_ADR, BME_REG_CONFIG, reg);
}

void setControlHumidRegister(BME_OVERSAMPLING humSamp){
    uint8_t reg = 0;

    reg = (uint8_t)humSamp;
    I2CWriteByte(BME_I2C_ADR, BME_REG_CTRL_HUM, reg);
}

void readTemperature(BME_DATA *data){
    int32_t var1, var2;
    int32_t adc_Temp;
    uint8_t buffer[3];

    I2CReadBytes(BME_I2C_ADR, BME_REG_TEMP_MSB, buffer, 3, TIMEOUT_I2C);

    adc_Temp = ((uint32_t)buffer[0] << 12) | ((uint16_t)buffer[1] << 4) | ((uint8_t)buffer[2] >> 4);

    var1 = (int32_t)((adc_Temp / 8) - ((int32_t)calibrationValues->T1 * 2));
    var1 = (var1 * ((int32_t)calibrationValues->T2)) / 2048;
    var2 = (int32_t)((adc_Temp / 16) - ((int32_t)calibrationValues->T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calibrationValues->T3)) / 16384;

    data->tfine = var1 + var2 + data->tfineAdjust;

    int32_t T = (data->tfine * 5 + 128) / 256;

    data->temp = (float)T / 100.0;
}

uint8_t readPressure(BME_DATA *data){
    int64_t var1, var2, var3, var4;
    int32_t adc_Press;
    uint8_t buffer[3];
    
    I2CReadBytes(BME_I2C_ADR, BME_REG_PRESS_MSB, buffer, 3, TIMEOUT_I2C);

    adc_Press = ((uint32_t)buffer[0] << 12) | ((uint16_t)buffer[1] << 4) | ((uint8_t)buffer[2] >> 4);

    var1 = ((int64_t)data->tfine) - 128000;
    var2 = var1 * var1 * (int64_t)calibrationValues->P6;
    var2 = var2 + ((var1 * (int64_t)calibrationValues->P5) * 131072);
    var2 = var2 + (((int64_t)calibrationValues->P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calibrationValues->P3) / 256) +
           ((var1 * ((int64_t)calibrationValues->P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)calibrationValues->P1) / 8589934592;

    if(var1 == 0){
        return 0;
    }

    var4 = 1048576 - adc_Press;
    var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
    var1 = (((int64_t)calibrationValues->P9) * (var4 / 8192) * (var4 / 8192)) /
           33554432;
    var2 = (((int64_t)calibrationValues->P8) * var4) / 524288;
    var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calibrationValues->P7) * 16);

    float P = var4 / 256.0;

    data->press = (float)P;
    return 0;
}

void readHumidity(BME_DATA *data){
    int32_t var1, var2, var3, var4, var5;
    int32_t adc_Hum;
    uint8_t buffer[2];

    I2CReadBytes(BME_I2C_ADR, BME_REG_HUM_MSB, buffer, 2, TIMEOUT_I2C);

    adc_Hum = ((uint16_t)buffer[0] << 8) | (uint8_t)buffer[1];

    var1 = data->tfine - ((int32_t)76800);
    var2 = (int32_t)(adc_Hum * 16384);
    var3 = (int32_t)(((int32_t)calibrationValues->H4) * 1048576);
    var4 = ((int32_t)calibrationValues->H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calibrationValues->H6)) / 1024;
    var3 = (var1 * ((int32_t)calibrationValues->H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calibrationValues->H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calibrationValues->H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    uint32_t H = (uint32_t)(var5 / 4096);


    data->hum = (float)H / 1024.0;
}

void readAltitude(BME_DATA *data, float sealevel){
    float atmospheric = data->press / 100.0F;

    data->altitude = 44330.0 * (1.0 - powf(atmospheric / sealevel, 0.1903));
}

void BMEGetData(BME_DATA *data){
    readTemperature(data);

    readPressure(data);

    readHumidity(data);
    
    readAltitude(data, SeaLevel);
}