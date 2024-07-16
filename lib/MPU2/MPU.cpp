#include "MPU.h"
#include "MPURegister.h"

// Tüm Register Ayarlamalarını Yapan Fonksiyondur
Status mpuInit(struct MPU_REGISTERS *settings){
    mpuSetPowerMngmtRegister(settings);
    delay(20);

    mpuSetUserControlRegister(settings);
    delay(20);

    mpuSetIntPinRegister(settings);
    delay(20);

    mpuSetI2CMasterRegister(settings);
    delay(20);

    mpuSetConfigRegister(settings);
    delay(20);

    mpuSetGyroConfigRegister(settings);
    delay(20);

    mpuSetAccConfigRegister(settings);
    delay(20);

    return MPU_Success;
}

// Çözünürlük ayarlamalarını yapar.
Status setScalingFactors(struct MPU_REGISTERS *setting){
    setGyroScalingFactor(setting);
    delay(10);
    setAccelScalingFactor(setting);
    delay(10);
    setMagScalingFactor(setting);
    delay(10);
    return MPU_Success;
}

// Ham ivme, gyro ve sıcaklık verilerini alır.
Status getRawAccGyroTempData(Dof3Data_Int *rawDataAccel, Dof3Data_Int *rawDataGyro, int16_t *temp){
    uint8_t buffer[14];
    I2CReadBytes(MPU_CHIPADR, ACCEL_X_OUTPUT_MSB, buffer, 14, TIMEOUT_I2C);

    rawDataAccel->x = (((int16_t)buffer[0] << 8) | buffer[1]);
    rawDataAccel->y = (((int16_t)buffer[2] << 8) | buffer[3]);
    rawDataAccel->z = (((int16_t)buffer[4] << 8) | buffer[5]);

    rawDataGyro->x = (((int16_t)buffer[8] << 8) | buffer[9]);
    rawDataGyro->y = (((int16_t)buffer[10] << 8) | buffer[11]);
    rawDataGyro->z = (((int16_t)buffer[12] << 8) | buffer[13]);

    *temp = (((int16_t)buffer[6] << 8) | buffer[7]);

    return MPU_Success;
}

// İvme, gyro ve sicaklik verilerini işler.
Status normalizeAccGyroTempData(Dof3Data_Int *rawDataAccel, Dof3Data_Int *rawDataGyro, int16_t *tempint, float *temp, Dof3Data_Float *accel, Dof3Data_Float *gyro){
    *temp = (float)(*tempint) / 333.87f + 21.0f;

    accel->x = (float)rawDataAccel->x / Acc_Resolution;
    accel->y = (float)rawDataAccel->y / Acc_Resolution;
    accel->z = (float)rawDataAccel->z / Acc_Resolution;

    gyro->x = (float)rawDataGyro->x / Gyro_Resolution;
    gyro->y = (float)rawDataGyro->y / Gyro_Resolution;
    gyro->z = (float)rawDataGyro->z / Gyro_Resolution;

    return MPU_Success;
}

// Mag verilerini alır.
Status getMagData(Dof3Data_IntMAG *data, Dof3Data_Float *magdata){
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

    

    return MPU_Success;
}