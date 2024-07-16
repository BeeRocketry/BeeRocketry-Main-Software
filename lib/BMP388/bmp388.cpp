#include "bmp388.h"
#include "debugprinter.h"

BMP_CalibData *CalibDataPointer;
BMP_QuantizedCalibData *QuantizedDataPointer;

void BMPInit(BMP_Oversampling pressureOversampling, BMP_Oversampling temperatureOversampling, BMP_IIR_Sampling iirSampling, BMP_ODR odrSampling){
    DEBUG_PRINTLN(F("------------"));
    DEBUG_PRINTLN(F("   BMP388"));
    DEBUG_PRINTLN(F("------------"));
    setReset();
    DEBUG_PRINTLN("BMP388 Resetlendi...");

    getCalibrationData();
    DEBUG_PRINTLN(F("Kalibrasyon değerleri başariyla alindi..."));

    setOSR(pressureOversampling, temperatureOversampling);
    setODR(odrSampling);

    DEBUG_PRINTLN(F("Oversampling ve ODR başariyla ayarlandi..."));

    setControl(iirSampling);

    setFIFOConfig1(BMP_OFF);
    setPowerControl(BMP_ON, BMP_ON, BMP_NormalMode);

    DEBUG_PRINTLN(F("BMP388 Sicaklik ve Basinc Olcumu Baslatildi..."));
}

void getCalibrationData(){
    CalibDataPointer = (BMP_CalibData *)malloc(sizeof(BMP_CalibData));
    QuantizedDataPointer = (BMP_QuantizedCalibData *)malloc(sizeof(BMP_QuantizedCalibData));

    uint8_t buffer[21];

    I2CReadBytes(CHIP_ADR, REG_TEMP_T1_LSB, buffer, 21, TIMEOUT_I2C);

    CalibDataPointer->T1 = ((uint16_t)buffer[1] << 8) | buffer[0];
    CalibDataPointer->T2 = ((uint16_t)buffer[3] << 8) | buffer[2];
    CalibDataPointer->T3 = (int8_t)buffer[4];

    CalibDataPointer->P1 = ((int16_t)buffer[6] << 8) | buffer[5];
    CalibDataPointer->P2 = ((int16_t)buffer[8] << 8) | buffer[7];
    CalibDataPointer->P3 = (int8_t)buffer[9];
    CalibDataPointer->P4 = (int8_t)buffer[10];
    CalibDataPointer->P5 = ((uint16_t)buffer[12] << 8) | buffer[11];
    CalibDataPointer->P6 = ((uint16_t)buffer[14] << 8) | buffer[13];
    CalibDataPointer->P7 = (int8_t)buffer[15];
    CalibDataPointer->P8 = (int8_t)buffer[16];
    CalibDataPointer->P9 = ((int16_t)buffer[18] << 8) | buffer[17];
    CalibDataPointer->P10 = (int8_t)buffer[19];
    CalibDataPointer->P11 = (int8_t)buffer[20];

    QuantizedDataPointer->T1 = (double)CalibDataPointer->T1 / 2e-8;
    QuantizedDataPointer->T2 = (double)CalibDataPointer->T2 / 2e30;
    QuantizedDataPointer->T3 = (double)CalibDataPointer->T3 / 2e48;

    QuantizedDataPointer->P1 = ((double)CalibDataPointer->P1 - 2e14) / 2e20;
    QuantizedDataPointer->P2 = ((double)CalibDataPointer->P2 - 2e14) / 2e29;
    QuantizedDataPointer->P3 = (double)CalibDataPointer->P3 / 2e32;
    QuantizedDataPointer->P4 = (double)CalibDataPointer->P4 / 2e37;
    QuantizedDataPointer->P5 = (double)CalibDataPointer->P5 / 2e-3;
    QuantizedDataPointer->P6 = (double)CalibDataPointer->P6 / 2e6;
    QuantizedDataPointer->P7 = (double)CalibDataPointer->P7 / 2e8;
    QuantizedDataPointer->P8 = (double)CalibDataPointer->P8 / 2e15;
    QuantizedDataPointer->P9 = (double)CalibDataPointer->P9 / 2e48;
    QuantizedDataPointer->P10 = (double)CalibDataPointer->P10 / 2e48;
    QuantizedDataPointer->P11 = (double)CalibDataPointer->P11 / 2e65;

    free(CalibDataPointer);
    CalibDataPointer = NULL;
}

/*
    FIFO mod ayarlamasını yapar.
*/
void setFIFOConfig1(BMP_ONOFF fifoMode){
    uint8_t temp = 0;

    temp = ((uint8_t)fifoMode) | temp;
    I2CWriteByte(CHIP_ADR, REG_FIFO_CONFIG1, temp);
}

/* 
    Power Control registerını düzenler
        PressureEnable --> 0.Bit
        TempEnable --> 1.Bit
        bmpMode --> 4 ve 5.Bit
*/
void setPowerControl(BMP_ONOFF pressureEnable, BMP_ONOFF tempEnable, BMP_Mode bmpMode){
    uint8_t temp = 0;

    temp |= ((uint8_t)bmpMode << 4) | ((uint8_t)tempEnable << 1) | (uint8_t)pressureEnable;
    I2CWriteByte(CHIP_ADR, REG_PWR_CTRL, temp);
}

/*
    Oversampling registerını düzenler.
        PressureOversampling --> 0, 1 ve 2.Bit
        TempOversampling --> 3, 4 ve 5.Bit
*/
void setOSR(BMP_Oversampling pressureOversampling, BMP_Oversampling tempOversampling){
    uint8_t temp = 0;

    temp |= ((uint8_t)tempOversampling << 3) | (uint8_t)pressureOversampling;
    I2CWriteByte(CHIP_ADR, REG_OSR, temp);
}

/*
    IIR Filter'ı kontrol eder.
        iirSampling --> 1, 2 ve 3.Bit
*/
void setControl(BMP_IIR_Sampling iirSampling){
    uint8_t temp = 0;

    temp |= ((uint8_t)iirSampling << 1);
    I2CWriteByte(CHIP_ADR, REG_CONFIG, temp);
}

/*
    Sampling Rate'i kontrol eder.
        odrRate --> 0, 1, 2, 3 ve 4.Bit
*/
void setODR(BMP_ODR odrRate){
    uint8_t temp = 0;

    temp |= (uint8_t)odrRate;
    I2CWriteByte(CHIP_ADR, REG_ODR, temp);
}

void setReset(){
    I2CWriteByte(CHIP_ADR, REG_CMD, 0xB6);
    delay(20);
}

void getRawData(uint32_t *rawpress, uint32_t *rawtemperature){
    uint8_t buffer[6];
    uint32_t temppres = 0, temptemp = 0;

    I2CReadBytes(CHIP_ADR, REG_PRESS_XLSB, buffer, 6, TIMEOUT_I2C);

    temppres = ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0];
    temptemp = ((uint32_t)buffer[5] << 16) | ((uint32_t)buffer[4] << 8) | buffer[3];

    *rawpress = temppres;
    *rawtemperature = temptemp;
}

float compensatedTempData(uint32_t rawTemperature){
    float tempData1, tempData2;

    tempData1 = (float)(rawTemperature - QuantizedDataPointer->T1);
    tempData2 = (float)(tempData1 * QuantizedDataPointer->T2);

    QuantizedDataPointer->tfine = tempData2 + (tempData1 * tempData1) * QuantizedDataPointer->T3;

    return QuantizedDataPointer->tfine;
}

float compensatedPressData(uint32_t rawPressure){
    float compPress;

    float tempData1, tempData2, tempData3, tempData4;
    float tempOut1, tempOut2;

    tempData1 = QuantizedDataPointer->P6 * QuantizedDataPointer->tfine;
    tempData2 = QuantizedDataPointer->P7 * (QuantizedDataPointer->tfine * QuantizedDataPointer->tfine);
    tempData3 = QuantizedDataPointer->P8 * (QuantizedDataPointer->tfine * QuantizedDataPointer->tfine * QuantizedDataPointer->tfine);
    tempOut1 = QuantizedDataPointer->P5 + tempData1 + tempData2 + tempData3;

    tempData1 = QuantizedDataPointer->P2 * QuantizedDataPointer->tfine;
    tempData2 = QuantizedDataPointer->P3 * (QuantizedDataPointer->tfine * QuantizedDataPointer->tfine);
    tempData3 = QuantizedDataPointer->P4 * (QuantizedDataPointer->tfine * QuantizedDataPointer->tfine * QuantizedDataPointer->tfine);
    tempOut2 = (float)rawPressure * (QuantizedDataPointer->P1 + tempData1 + tempData2 + tempData3);

    tempData1 = (float)rawPressure * (float)rawPressure;
    tempData2 = QuantizedDataPointer->P9 + QuantizedDataPointer->P10 * QuantizedDataPointer->tfine;
    tempData3 = tempData1 * tempData2;
    tempData4 = tempData3 + ((float)rawPressure * (float)rawPressure * (float)rawPressure) * QuantizedDataPointer->P11;
    compPress = tempOut1 + tempOut2 + tempData4;

    return compPress;
}

float convertPress2Altitude(float pressure){
    double Tb = 288.15;
    double Lb = 0.0065;
    double Pb = SeaLevelhPa * 100;
    double exp = 1.0 / 5.255;
    double fac = Tb / Lb;

    float altitude = fac * (1 - pow((float)(pressure / Pb), (float)exp));

    return altitude;
}

// Tüm BMP388 Dataları Alacak Fonksiyon
void BMPGetData(float *temperature, float *pressure, float *altitude){
    uint32_t rawPress, rawTemp;
    float realTemperature, realPressure, realAltitude;

    getRawData(&rawPress, &rawTemp);

    realTemperature = compensatedTempData(rawTemp);
    realPressure = compensatedPressData(rawPress);
    realAltitude = convertPress2Altitude(realPressure);
    
    *temperature = realTemperature;
    *pressure = realPressure;
    *altitude = realAltitude;
}