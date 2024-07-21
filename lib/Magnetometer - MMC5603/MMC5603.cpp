#include "MMC5603.h"

// Magnetometer Control Register Ayarlamasini Yapar
void MMCBegin(bool continuousmode, uint16_t datarate){
    DEBUG_PRINTLN(F("MMC5603 Register Ayarlamalarina Baslaniyor..."));
    uint8_t ctrl2;
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