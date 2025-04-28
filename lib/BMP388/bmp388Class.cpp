#include "bmp388Class.h"

void bmp388::getCalibrationData(){
    if(this->CalibDataPointer == nullptr) this->CalibDataPointer = new BMP_CalibData;
    if(this->QuantizedDataPointer == nullptr) this->QuantizedDataPointer = new BMP_QuantizedCalibData;

    memset(this->CalibDataPointer, 0, sizeof(BMP_CalibData));
    memset(this->QuantizedDataPointer, 0, sizeof(BMP_QuantizedCalibData));

    uint8_t buffer[21];

    I2CReadBytes(BMP_CHIPADR, REG_TEMP_T1_LSB, buffer, 21, TIMEOUT_I2C);

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

    QuantizedDataPointer->T1 = (double)CalibDataPointer->T1 / 0.00390625f;
    QuantizedDataPointer->T2 = (double)CalibDataPointer->T2 / 1073741824.0f;
    QuantizedDataPointer->T3 = (double)CalibDataPointer->T3 / 281474976710656.0f;

    QuantizedDataPointer->P1 = ((double)CalibDataPointer->P1 - 16384) / 1048576.0f;
    QuantizedDataPointer->P2 = ((double)CalibDataPointer->P2 - 16384) / 536870912.0f;
    QuantizedDataPointer->P3 = (double)CalibDataPointer->P3 / 4294967296.0f;
    QuantizedDataPointer->P4 = (double)CalibDataPointer->P4 / 137438953472.0f;
    QuantizedDataPointer->P5 = (double)CalibDataPointer->P5 / 0.125f;
    QuantizedDataPointer->P6 = (double)CalibDataPointer->P6 / 64.0f;
    QuantizedDataPointer->P7 = (double)CalibDataPointer->P7 / 256.0f;
    QuantizedDataPointer->P8 = (double)CalibDataPointer->P8 / 32768.0f;
    QuantizedDataPointer->P9 = (double)CalibDataPointer->P9 / 281474976710656.0f;
    QuantizedDataPointer->P10 = (double)CalibDataPointer->P10 / 281474976710656.0f;
    QuantizedDataPointer->P11 = (double)CalibDataPointer->P11 / 36893488147419103232.0f;

    delete this->CalibDataPointer;
    CalibDataPointer = nullptr;
}

uint8_t bmp388::bmpReadByte(uint8_t regadr, uint8_t* temp, uint16_t timeout = 1000){
    return this->_i2c->I2CReadByte(BMP_CHIPADR, regadr, temp, timeout);
}

uint8_t bmp388::bmpReadBytes(uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout = 1000){
    return this->_i2c->I2CReadBytes(BMP_CHIPADR, regadr, temp, length, timeout);
}

bool bmp388::bmpWriteByte(uint8_t regadr, uint8_t data){
    return this->_i2c->I2CWriteByte(BMP_CHIPADR, regadr, data);
}

uint32_t bmp388::getTempRaw(){
    uint8_t buffer[3] = {0};
    uint32_t temp = 0;

    this->bmpReadBytes(REG_TEMP_XLSB, buffer, 3);

    temp = ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0];

    return temp;
}

uint32_t bmp388::getPresRaw(){
    uint8_t buffer[3] = {0};
    uint32_t pres = 0;

    this->bmpReadBytes(REG_PRESS_XLSB, buffer, 3);

    pres = ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0];

    return pres;
}

/*
    FIFO mod ayarlamasını yapar.
*/
void bmp388::setFIFOConfig1(BMP_ONOFF fifoMode){
    uint8_t temp = 0;

    temp = ((uint8_t)fifoMode) | temp;
    this->bmpWriteByte(REG_FIFO_CONFIG1, temp);
}


/* 
    Power Control registerını düzenler
        PressureEnable --> 0.Bit
        TempEnable --> 1.Bit
        bmpMode --> 4 ve 5.Bit
*/
void bmp388::setPowerControl(BMP_ONOFF pressureEnable, BMP_ONOFF tempEnable, BMP_Mode bmpMode){
    uint8_t temp = 0;

    temp |= ((uint8_t)bmpMode << 4) | ((uint8_t)tempEnable << 1) | (uint8_t)pressureEnable;
    this->bmpWriteByte(REG_PWR_CTRL, temp);
}

/*
    Oversampling registerını düzenler.
        PressureOversampling --> 0, 1 ve 2.Bit
        TempOversampling --> 3, 4 ve 5.Bit
*/
void bmp388::setOSR(BMP_Oversampling pressureOversampling, BMP_Oversampling tempOversampling){
    uint8_t temp = 0;

    temp |= ((uint8_t)tempOversampling << 3) | (uint8_t)pressureOversampling;
    this->bmpWriteByte(REG_OSR, temp);
}

/*
    IIR Filter'ı kontrol eder.
        iirSampling --> 1, 2 ve 3.Bit
*/
void bmp388::setControl(BMP_IIR_Sampling iirSampling){
    uint8_t temp = 0;

    temp |= ((uint8_t)iirSampling << 1);
    this->bmpWriteByte(REG_CONFIG, temp);
}

/*
    Sampling Rate'i kontrol eder.
        odrRate --> 0, 1, 2, 3 ve 4.Bit
*/
void bmp388::setODR(BMP_ODR odrRate){
    uint8_t temp = 0;

    temp |= (uint8_t)odrRate;
    this->bmpWriteByte(REG_ODR, temp);
}

void bmp388::setReset(){
    this->bmpWriteByte(REG_CMD, 0xB6);
    delay(20);
}

bmp388::bmp388(I2Class* _i2c) : _i2c(_i2c){
    this->BMPInit(BMP_OverSampling_4x, BMP_OverSampling_2x, BMP_IIR_OFF, BMP_ODR_80ms);
}

bmp388::bmp388(I2Class* _i2c, BMP_Oversampling pressureOversampling, BMP_Oversampling temperatureOversampling, BMP_IIR_Sampling iirSampling, BMP_ODR odrSampling)  : _i2c(_i2c){
    this->BMPInit(pressureOversampling, temperatureOversampling, iirSampling, odrSampling);
}

bmp388::~bmp388(){
    delete this->QuantizedDataPointer;
}

void bmp388::BMPInit(BMP_Oversampling pressureOversampling, BMP_Oversampling temperatureOversampling, BMP_IIR_Sampling iirSampling, BMP_ODR odrSampling){
    DEBUG_PRINTLN(F("------------"));
    DEBUG_PRINTLN(F("   BMP388"));
    DEBUG_PRINTLN(F("------------"));
    setReset();
    DEBUG_PRINTLN("BMP388 Resetlendi...");

    delay(20);

    getCalibrationData();
    DEBUG_PRINTLN(F("Kalibrasyon değerleri başariyla alindi..."));

    setOSR(pressureOversampling, temperatureOversampling);
    setControl(iirSampling);
    setODR(odrSampling);
    setFIFOConfig1(BMP_OFF);
    setPowerControl(BMP_ON, BMP_ON, BMP_NormalMode);
    
    DEBUG_PRINTLN(F("Oversampling ve ODR başariyla ayarlandi..."));

    DEBUG_PRINTLN(F("BMP388 Sicaklik ve Basinc Olcumu Baslatildi..."));
}

float bmp388::getTempData(){
    uint32_t tempRaw = this->getTempRaw();

    float tempData1, tempData2;

    tempData1 = (float)(tempRaw - this->QuantizedDataPointer->T1);
    tempData2 = (float)(tempData1 * this->QuantizedDataPointer->T2);

    QuantizedDataPointer->tfine = tempData2 + (tempData1 * tempData1) * QuantizedDataPointer->T3;

    return QuantizedDataPointer->tfine;
}

float bmp388::getPresData(){
    this->getTempData();
    uint32_t presRaw = this->getPresRaw();
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
    tempOut2 = (float)presRaw * (QuantizedDataPointer->P1 + tempData1 + tempData2 + tempData3);

    tempData1 = (float)presRaw * (float)presRaw;
    tempData2 = QuantizedDataPointer->P9 + QuantizedDataPointer->P10 * QuantizedDataPointer->tfine;
    tempData3 = tempData1 * tempData2;
    tempData4 = tempData3 + ((float)presRaw * (float)presRaw * (float)presRaw) * QuantizedDataPointer->P11;
    compPress = tempOut1 + tempOut2 + tempData4;

    return compPress;
}

float bmp388::getAltitude(float pressure){
    double Tb = 288.15;
    double Lb = 0.0065;
    double Pb = SeaLevelhPa * 100;
    double exp = 1.0 / 5.255;
    double fac = Tb / Lb;

    float altitude = fac * (1 - pow((float)(pressure / Pb), (float)exp));

    return altitude;
}

BMP_SensorData bmp388::BMPGetData(){
    uint32_t rawPress, rawTemp;
    BMP_SensorData sensorData;

    memset(&sensorData, 0, sizeof(BMP_SensorData));

    sensorData.temperature = this->getTempData();
    sensorData.pressure = this->getPresData();
    sensorData.altitude = this->getAltitude(sensorData.pressure);

    return sensorData;
}

uint8_t bmp388::getBMPChipID(){
    uint8_t reg = 0;

    this->bmpReadByte(0x00, &reg);
    return reg;
}