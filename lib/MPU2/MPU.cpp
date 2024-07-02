#include "MPU.h"
#include "MPURegister.h"

// Çözünürlük ayarlamalarını yapar.
Status setScalingFactors(struct MPU_REGISTERS *settings){
    setGyroScalingFactor(settings);
    delay(10);
    setAccelScalingFactor(settings);
    delay(10);
    setMagScalingFactor(settings);
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

    accel->x = (float)rawDataAccel->x / Acc_Resolution - Acc_bias[0];
    accel->y = (float)rawDataAccel->y / Acc_Resolution - Acc_bias[1];
    accel->z = (float)rawDataAccel->z / Acc_Resolution - Acc_bias[2];

    gyro->x = (float)rawDataGyro->x / Gyro_Resolution - Gyro_bias[0];
    gyro->y = (float)rawDataGyro->y / Gyro_Resolution - Gyro_bias[1];
    gyro->z = (float)rawDataGyro->z / Gyro_Resolution - Gyro_bias[2];

    return MPU_Success;
}

// Ham mag verilerini alır.
Status getMagData(Dof3Data_Int *data){
    uint8_t buffer[6];
    I2CReadBytes(MAG_CHIPADR, MAG_X_OUTPUT_LSB, buffer, 6, TIMEOUT_I2C);

    data->x = (((int16_t)buffer[1] << 8) | buffer[0]);
    data->y = (((int16_t)buffer[3] << 8) | buffer[2]);
    data->z = (((int16_t)buffer[5] << 8) | buffer[4]);

    return MPU_Success;
}


// Ham mag verilerini işler ve dönüştürür.
Status normalizeMagData(Dof3Data_Int *data, Dof3Data_Float *mag){
    float bias_to_current_bits = Mag_Resolution * getMagRes(MAG_OUTPUTBIT_16BIT);

    mag->x = (float)(data->x * Mag_Resolution * Mag_bias_factory[0] - Mag_bias[0] * bias_to_current_bits) * Mag_scale[0];
    mag->y = (float)(data->y * Mag_Resolution * Mag_bias_factory[1] - Mag_bias[1] * bias_to_current_bits) * Mag_scale[1];
    mag->z = (float)(data->z * Mag_Resolution * Mag_bias_factory[2] - Mag_bias[2] * bias_to_current_bits) * Mag_scale[2];

    DEBUG_PRINTLN(F("Mag"));
    DEBUG_PRINT(F(" X:"));
    DEBUG_PRINTLN(F(mag->x));
    DEBUG_PRINT(F(" Y:"));
    DEBUG_PRINTLN(F(mag->y));
    DEBUG_PRINT(F(" Z:"));
    DEBUG_PRINTLN(F(mag->z));
    DEBUG_PRINTLN();

    return MPU_Success;
}