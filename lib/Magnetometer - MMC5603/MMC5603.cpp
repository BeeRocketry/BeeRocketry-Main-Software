#include "MMC5603.h"

float magHighCalValue[3] = { -3.98 , -5.29 , 27.28 };
float magSoftCalValue[3][3] = { { 1.018 , -0.012 , 0.030 },
                                { -0.012 , 0.978 , 0.008 },
                                { 0.030 , 0.008 , 1.006 }};


// Magnetometer Control Register Ayarlamasini Yapar
void MMCBegin(bool continuousmode, uint16_t datarate){
    DEBUG_PRINTLN(F("MMC5603 Register Ayarlamalarina Baslaniyor..."));

    I2CWriteByte(MMC5603_CHIPADR, MMC_Control1, 0b10000000);
    delay(30);
    uint8_t ctrl1, ctrl2;

    I2CReadByte(MMC5603_CHIPADR, MMC_Control1, &ctrl1, I2C_TIMEOUT);
    ctrl1 |= 0b11;
    I2CWriteByte(MMC5603_CHIPADR, MMC_Control1, ctrl1);

    I2CReadByte(MMC5603_CHIPADR, MMC_Control2, &ctrl2, I2C_TIMEOUT);
    if(continuousmode){
        I2CWriteByte(MMC5603_CHIPADR, MMC_Control0, 0x80);
        ctrl2 |= 0x10;
    }
    else{
        ctrl2 &= ~0x10;
    }
    I2CWriteByte(MMC5603_CHIPADR, MMC_Control2, ctrl2);

    DEBUG_PRINTLN(F("MMC5603 Mod Ayarlamasi Yapildi..."));

    delay(50);

    if(datarate > 255){
        datarate = 1000;
    }

    if(datarate == 1000){
        I2CWriteByte(MMC5603_CHIPADR, MMC_ODR, 255);
        ctrl2 |= 0x80;
        I2CWriteByte(MMC5603_CHIPADR, MMC_Control2, ctrl2);
    }
    else{
        I2CWriteByte(MMC5603_CHIPADR, MMC_ODR, datarate);
        ctrl2 &= ~0x80;
        I2CWriteByte(MMC5603_CHIPADR, MMC_Control2, ctrl2);
    }

    DEBUG_PRINTLN(F("MMC5603 ODR Ayarlamasi Yapildi..."));
}

// Mag verilerini alır.
void getMagData(Dof3Data_IntMAG *data, Dof3Data_FloatMMC *magdata){
    uint8_t buffer[9];
    I2CReadBytes(MMC5603_CHIPADR, MMC_X_OUTPUT_MSB, buffer, 9, TIMEOUT_I2C);

    data->x = (uint32_t)buffer[0] << 12 | (uint32_t)buffer[1] << 4 | (uint32_t)buffer[6] >> 4;
    data->y = (uint32_t)buffer[2] << 12 | (uint32_t)buffer[3] << 4 | (uint32_t)buffer[7] >> 4;
    data->z = (uint32_t)buffer[4] << 12 | (uint32_t)buffer[5] << 4 | (uint32_t)buffer[8] >> 4;

    data->x -= (uint32_t)1 << 19;
    data->y -= (uint32_t)1 << 19;
    data->z -= (uint32_t)1 << 19;

    magdata->x = (float)data->x * 0.00625;
    magdata->y = (float)data->y * 0.00625;
    magdata->z = (float)data->z * 0.00625;
}

void getRawMagData(Dof3Data_IntMAG *data){
    uint8_t buffer[9];
    I2CReadBytes(MMC5603_CHIPADR, MMC_X_OUTPUT_MSB, buffer, 9, TIMEOUT_I2C);

    data->x = (uint32_t)buffer[0] << 12 | (uint32_t)buffer[1] << 4 | (uint32_t)buffer[6] >> 4;
    data->y = (uint32_t)buffer[2] << 12 | (uint32_t)buffer[3] << 4 | (uint32_t)buffer[7] >> 4;
    data->z = (uint32_t)buffer[4] << 12 | (uint32_t)buffer[5] << 4 | (uint32_t)buffer[8] >> 4;

    data->x -= (uint32_t)1 << 19;
    data->y -= (uint32_t)1 << 19;
    data->z -= (uint32_t)1 << 19;
}

void compensatedMagData(Dof3Data_IntMAG data, Dof3Data_FloatMMC *magdata){
    magdata->x = (float)data.x * 0.00625;
    magdata->y = (float)data.y * 0.00625;
    magdata->z = (float)data.z * 0.00625;
}

void getHighCalibrated(Dof3Data_IntMAG *magData){
    magData->x -= magHighCalValue[0];
    magData->y -= magHighCalValue[1];
    magData->z -= magHighCalValue[2];
}

void vectorNormalize(Dof3Data_IntMAG *magData){
    float mag = sqrtf(vectorDotProduct(magData, magData));
    DEBUG_PRINT(F("Mag: "));
    DEBUG_PRINTLN(mag);

    magData->x /= mag;
    magData->y /= mag;
    magData->z /= mag;
}

float vectorDotProduct(Dof3Data_IntMAG *a, Dof3Data_IntMAG *b){
    return ((a->x * b->x) + (a->y * b->y) + (a->z * b->z));
}

void getSoftCalibrateda(Dof3Data_IntMAG *magData){
    int32_t temp[3];
    temp[0] = magData->x;
    temp[1] = magData->y;
    temp[2] = magData->z;

    magData->x = magSoftCalValue[0][0] * (float)temp[0] + magSoftCalValue[0][1] * (float)temp[1] + magSoftCalValue[0][2] * (float)temp[2];
    magData->y = magSoftCalValue[1][0] * (float)temp[0] + magSoftCalValue[1][1] * (float)temp[1] + magSoftCalValue[1][2] * (float)temp[2];
    magData->z = magSoftCalValue[2][0] * (float)temp[0] + magSoftCalValue[2][1] * (float)temp[1] + magSoftCalValue[2][2] * (float)temp[2];
    //vectorNormalize(magData);
}