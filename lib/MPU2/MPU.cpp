#include "MPU.h"

int16_t gyroffset[3] = {0};

// Tüm Register Ayarlamalarını Yapan Fonksiyondur
MPU_Status mpuInit(struct MPU_REGISTERS *settings){
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, 0b10000000);
    delay(40);

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

    setScalingFactors(settings);
    delay(20);

    mpuSetPowerMngmtRegister(settings);
    delay(20);

    return MPU_Success;
}

// Çözünürlük ayarlamalarını yapar.
MPU_Status setScalingFactors(struct MPU_REGISTERS *setting){
    setGyroScalingFactor(setting);
    delay(10);
    setAccelScalingFactor(setting);
    delay(10);
    return MPU_Success;
}

// Ham ivme, gyro ve sıcaklık verilerini alır.
MPU_Status getRawAccGyroTempData(Dof3Data_Int *rawDataAccel, Dof3Data_Int *rawDataGyro, int16_t *temp){
    uint8_t buffer[14];
    I2CReadBytes(MPU_CHIPADR, ACCEL_X_OUTPUT_MSB, buffer, 14, TIMEOUT_I2C);

    rawDataAccel->x = (((int16_t)buffer[0] << 8) | (int16_t)buffer[1]);
    rawDataAccel->y = (((int16_t)buffer[2] << 8) | (int16_t)buffer[3]);
    rawDataAccel->z = (((int16_t)buffer[4] << 8) | (int16_t)buffer[5]);

    rawDataGyro->x = (((int16_t)buffer[8] << 8) | (int16_t)buffer[9]);
    rawDataGyro->y = (((int16_t)buffer[10] << 8) | (int16_t)buffer[11]);
    rawDataGyro->z = (((int16_t)buffer[12] << 8) | (int16_t)buffer[13]);

    *temp = (((int16_t)buffer[6] << 8) | (int16_t)buffer[7]);

    return MPU_Success;
}

// İvme, gyro ve sicaklik verilerini işler.
MPU_Status normalizeAccGyroTempData(Dof3Data_Int *rawDataAccel, Dof3Data_Int *rawDataGyro, int16_t *tempint, float *temp, Dof3Data_Float *accel, Dof3Data_Float *gyro){
    *temp = (float)(*tempint) / 333.87f + 21.0f;

    accel->x = (float)rawDataAccel->x / Acc_Resolution;
    accel->y = (float)rawDataAccel->y / Acc_Resolution;
    accel->z = (float)rawDataAccel->z / Acc_Resolution;

    gyro->x = (float)(rawDataGyro->x - gyroffset[0]) / Gyro_Resolution;
    gyro->y = (float)(rawDataGyro->y - gyroffset[1]) / Gyro_Resolution;
    gyro->z = (float)(rawDataGyro->z - gyroffset[2]) / Gyro_Resolution;

    return MPU_Success;
}

MPU_Status getAccGyroData(Dof3Data_Float *accel, Dof3Data_Float *gyro){
    Dof3Data_Int rawAcc, rawGyro;
    int16_t temp;
    float tempF;
    getRawAccGyroTempData(&rawAcc, &rawGyro, &temp);
    normalizeAccGyroTempData(&rawAcc, &rawGyro, &temp, &tempF, accel, gyro);
    
    return MPU_Success;
}

// Power Management Register Ayarlamasını Yapar
MPU_Status mpuSetPowerMngmtRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_PowerManagament1_Register.activationmode << 6));
    reg = (reg | ((uint8_t)settings->MPU_PowerManagament1_Register.clockSet));

    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, reg);

    return MPU_Success;
}

// Config Register Ayarlamasını Yapar
MPU_Status mpuSetConfigRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_Config_Register.fifoMode << 6));
    reg = (reg | ((uint8_t)settings->MPU_Config_Register.syncMode << 3));
    reg = (reg | ((uint8_t)settings->MPU_Config_Register.dlpfMode));

    I2CWriteByte(MPU_CHIPADR, CONFIG, reg);

    return MPU_Success;
}

// Gyro Config Register Ayarlamasını Yapar
MPU_Status mpuSetGyroConfigRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_GyroConfig_Register.gyroScale << 3));
    reg = (reg | ((uint8_t)settings->MPU_GyroConfig_Register.dlpfchoice));

    I2CWriteByte(MPU_CHIPADR, CONFIG_GYRO, reg);

    return MPU_Success;
}

// İvme Config Register Ayarlamasini Yapar
MPU_Status mpuSetAccConfigRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_AccConfig_Register.accScale << 3));

    I2CWriteByte(MPU_CHIPADR, CONFIG_ACCEL, reg);

    delay(20);

    reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_AccConfig_2_Register.accDLPFmode << 3));
    reg = (reg | ((uint8_t)settings->MPU_AccConfig_2_Register.accDLPFSettings));

    I2CWriteByte(MPU_CHIPADR, CONFIG_ACCEL_2, reg);

    return MPU_Success;
}

// I2C Master Register Ayarlamasını Yapar
MPU_Status mpuSetI2CMasterRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_I2CMaster_Register.masterClock));

    I2CWriteByte(MPU_CHIPADR, I2C_MASTER_CONTROL, reg);

    return MPU_Success;
}

// Interrupt Pin Register Ayarlamasini Yapar
MPU_Status mpuSetIntPinRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_INTPin_Register.i2cbypassMode << 1));

    I2CWriteByte(MPU_CHIPADR, INT_PIN_CFG, reg);

    return MPU_Success;
}

// User Control Register Ayarlamasini Yapar
MPU_Status mpuSetUserControlRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_UserControl_Register.fifoEnable << 6));
    reg = (reg | ((uint8_t)settings->MPU_UserControl_Register.i2cMasterEnable << 5));

    I2CWriteByte(MPU_CHIPADR, USER_CTRL, reg);

    DEBUG_PRINTLN(F("MPU USER Control Registeri Ayarlandi..."));

    return MPU_Success;
}

MPU_Status GyroCalibration(uint32_t numSample){
    Dof3Data_Int gyro, acc;
    int16_t temp;
    int gyromin[3] = {32767 , 32767 , 32767};
    int gyromax[3] = {-32767 , -32767 , -32767};
    int gyromean[3];
    for(int i = 0; i < numSample; i++){
        if(numSample % 100 == 0){
            DEBUG_PRINT(F("Sample "));
            DEBUG_PRINTLN(i);
        }

        getRawAccGyroTempData(&acc, &gyro, &temp);

        if(gyro.x > gyromax[0]){
            gyromax[0] = gyro.x;
        }
        if(gyro.x < gyromin[0]){
            gyromin[0] = gyro.x;
        }

        if(gyro.y > gyromax[1]){
            gyromax[1] = gyro.y;
        }
        if(gyro.y < gyromin[1]){
            gyromin[1] = gyro.y;
        }

        if(gyro.z > gyromax[2]){
            gyromax[2] = gyro.z;
        }
        if(gyro.z < gyromin[2]){
            gyromin[2] = gyro.z;
        }

        delay(20);
    }

    for(int i = 0; i < 3; i++){
        gyromean[i] = (gyromax[i] + gyromin[i]) / 2;
    }

    DEBUG_PRINTLN(F("Gyro Offsets"));
    DEBUG_PRINT(F("X: "));
    DEBUG_PRINT(gyromean[0]);
    DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Y: "));
    DEBUG_PRINT(gyromean[1]);
    DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Z: "));
    DEBUG_PRINT(gyromean[2]);

    gyroffset[0] = gyromean[0];
    gyroffset[1] = gyromean[1];
    gyroffset[2] = gyromean[2];

    /*while(1){

    }*/
   return MPU_Success;
}