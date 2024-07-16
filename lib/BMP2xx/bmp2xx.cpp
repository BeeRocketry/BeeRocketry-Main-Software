#include "debugprinter.h"
#include "bmp2xx.h"

// Başlangıç ayarlamalarını yapar.
void bmpInit(void)
{
    setCtrlReg(0b001, 0b011, 0b11);
    setConfig(0b010, 0b000, 0b0);
    DEBUG_PRINTLN(F("BMP Port Aktifleştirildi."));
    DEBUG_PRINT(F("BMP I2C Adress: 0x"));
    DEBUG_PRINTLN(bmpgetDeviceID());
}

uint8_t bmpgetDeviceID(void){
    uint8_t temp;
    I2CReadByte(CHIP_ADR, REG_ID, &temp, TIMEOUT_I2C);

    return temp;
}

void setCtrlReg(byte oversamplingTemp, byte oversamplingPressure, byte powerMode)
{
    byte temp = 0;
    temp = (oversamplingTemp << 5) | (oversamplingPressure << 2) | powerMode;
    I2CWriteByte(CHIP_ADR, REG_CTRL_MEAS, temp);
}

void setConfig(byte tStandby, byte filterSet, byte spi3w)
{
    byte temp = 0;
    temp = (tStandby << 5) | (filterSet << 2) | spi3w;
    I2CWriteByte(CHIP_ADR, REG_CONFIG, temp);
}

// Raw Sıcaklık Değerini Alır
int32_t getRawTemp(void)
{
    int i = 0;
    int error = 5;
    uint8_t temporary[3] = {0};
    I2CReadBytes(CHIP_ADR, REG_TEMP_XLSB, temporary, 3, TIMEOUT_I2C);

    int32_t temp = 0;
    temp = (temporary[2] << 12) | (temporary[1] << 4) | (temporary[0] >> 4);

    return temp;
}

// Gerçek Sıcaklık değerini alır
int32_t getCompensatedTemp(int32_t rawData, int32_t *tfine)
{
    int32_t var1, var2, T;
    unsigned short dig_T1 = 0;
    short dig_T2 = 0, dig_T3 = 0;

    getTempCalb(&dig_T1, &dig_T2, &dig_T3);

    var1 = (((rawData >> 3) - (dig_T1 << 1)) * dig_T2) >> 11;
    var2 = (((((rawData >> 4) - dig_T1) * ((rawData >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;

    *tfine = var1 + var2;
    T = (*tfine * 5 + 128) >> 8;

    return T;
}

void getTempCalb(unsigned short *T1, short *T2, short *T3)
{
    uint8_t buff[6];

    I2CReadBytes(CHIP_ADR, REG_T1_LSB, buff, 6, TIMEOUT_I2C);

    *T1 = (buff[1] << 8) | buff[0];
    *T2 = (buff[3] << 8) | buff[2];
    *T3 = (buff[5] << 8) | buff[4];
}

int32_t getRawPres(void)
{
    uint8_t temporary[3];
    I2CReadBytes(CHIP_ADR, REG_PRESS_XLSB, temporary, 3, TIMEOUT_I2C);

    int32_t press = 0;
    press = (temporary[2] << 12) | (temporary[1] << 4) | (temporary[0] >> 4);

    return press;
}

uint32_t getCompensatedPres(int32_t rawData, int32_t tfine)
{
    int32_t var1, var2;
    uint32_t P;
    unsigned short dig_P1 = 0;
    short dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;

    getPresCalb(&dig_P1, &dig_P2, &dig_P3, &dig_P4, &dig_P5, &dig_P6, &dig_P7, &dig_P8, &dig_P9);

    var1 = (tfine >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (dig_P6);
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * dig_P1) >> 15;

    if (var1 == 0)
    {
        return 0;
    }

    P = (((uint32_t)(((int32_t)1048576) - rawData) - (var2 >> 12))) * 3125;

    if (P < 0x80000000)
    {
        P = (P << 1) / ((uint32_t)var1);
    }
    else
    {
        P = (P / (uint32_t)var1) * 2;
    }

    var1 = (dig_P9 * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(P >> 2)) * dig_P8) >> 13;

    P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

void getPresCalb(unsigned short *P1, short *P2, short *P3, short *P4, short *P5, short *P6, short *P7, short *P8, short *P9)
{
    uint8_t buff[18];

    I2CReadBytes(CHIP_ADR, REG_P1_LSB, buff, 18, TIMEOUT_I2C);

    for (int i = 0; i < 9; i++)
    {
        int32_t value = 0;
        value = (buff[i * 2 + 1] << 8) | buff[i * 2];
        switch (i)
        {
        case 0:
            *P1 = value;
            break;
        case 1:
            *P2 = value;
            break;
        case 2:
            *P3 = value;
            break;
        case 3:
            *P4 = value;
            break;
        case 4:
            *P5 = value;
            break;
        case 5:
            *P6 = value;
            break;
        case 6:
            *P7 = value;
            break;
        case 7:
            *P8 = value;
            break;
        case 8:
            *P9 = value;
            break;
        }
    }
}

// Hem sıcaklık hem de basınç raw verilerini alır
void getRaws(int32_t *pres, int32_t *temp)
{
    uint8_t temporary[6];
    I2CReadBytes(CHIP_ADR, REG_PRESS_MSB, temporary, 6, TIMEOUT_I2C);

    *pres = (temporary[0] << 12) | (temporary[1] << 4) | (temporary[0] >> 4);
    *temp = (temporary[3] << 12) | (temporary[4] << 4) | (temporary[5] >> 4);
}

// Yükseklik döndürür
float getAltitude(int32_t pressure, int32_t temperature)
{
    float preshpa, tempC, altitude;

    preshpa = pressure * 1.0 / 100;
    tempC = temperature * 1.0 / 100;
    altitude = 44330 * (1.0 - pow(preshpa / seaLevelhPa, 0.1903));
    
    return altitude;
}

// Test fonksiyonu
void bmpTest(int32_t *t, int32_t *p, float * a)
{
    int32_t rawTemp, rawPres, temp, pres;
    int32_t tfine;
    float altitude;
    float preshpa, tempc;

    getRaws(&rawPres, &rawTemp);

    DEBUG_PRINT(F("rawTemp: "));
    DEBUG_PRINTLN(rawTemp);
    DEBUG_PRINT(F("rawPres: "));
    DEBUG_PRINTLN(rawPres);

    temp = getCompensatedTemp(rawTemp, &tfine);
    pres = getCompensatedPres(rawPres, tfine);

    altitude = getAltitude(pres, temp);

    *a = altitude;
    *t = temp;
    *p = pres;
    DEBUG_PRINT(altitude);
    DEBUG_PRINTLN(F(" meter"));
    DEBUG_PRINTLN();
    DEBUG_PRINTLN();
}

// Tüm sıcaklık, basınc ve yükseklik verisini döndüren fonksiyon
float getAltitudeReal(int32_t *temp, int32_t *pres){
    int32_t tfine;
    getRaws(pres, temp);
    int32_t tempBuf = *temp;
    int32_t presBuf = *pres;

    *temp = getCompensatedTemp(tempBuf, &tfine);
    *pres = getCompensatedPres(presBuf, tfine);

    return getAltitude(*pres, *temp);
}