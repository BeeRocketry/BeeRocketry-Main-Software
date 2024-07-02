#include "MPU.h"
#include "MPURegister.h"

float Acc_Resolution = 0;
float AccRange = 0;
float Gyro_Resolution = 0;
float GyroRange = 0;

float Acc_bias[3] = {0, 0, 0};
float Gyro_bias[3] = {0, 0, 0};
const uint16_t CALIB_ACCEL_SENSIVITY = 16384;

// Gyro çözünürlüğünü verir.
Status setGyroScalingFactor(struct MPU_REGISTERS *settings){
    DEBUG_PRINTLN(F("Gyro Scaling Faktör Ayarlaniyor..."));
    switch (settings->MPU_GyroConfig_Register.gyroScale)
    {
    case 0b00:
        Gyro_Resolution = 131;
        GyroRange = 250;
        DEBUG_PRINT(F("131 "));
        break;
    
    case 0b01:
        Gyro_Resolution = 65.5;
        GyroRange = 500;
        DEBUG_PRINT(F("65.5 "));
        break;

    case 0b10:
        Gyro_Resolution = 32.8;
        GyroRange = 1000;
        DEBUG_PRINT(F("32.8 "));
        break;

    case 0b11:
        Gyro_Resolution = 16.4;
        GyroRange = 2000;
        DEBUG_PRINT(F("16.4 "));
        break;

    default:
        DEBUG_PRINTLN(F("Gyro Scaling Faktör girdisi yanlis..."));
        return MPU_Error;
    }
    DEBUG_PRINTLN(F("Gyro Scaling Faktör olarak ayarlandi..."));
    return MPU_Success;
}

// İvme çözünürlüğünü verir.
Status setAccelScalingFactor(struct MPU_REGISTERS *settings){
    DEBUG_PRINTLN(F("Accel Scaling Faktör Ayarlaniyor..."));
    switch (settings->MPU_AccConfig_Register.accScale)
    {
    case 0b00:
        Acc_Resolution = 16384;
        AccRange = 2;
        DEBUG_PRINT(F("16.384 "));
        break;
    
    case 0b01:
        Acc_Resolution = 8192;
        AccRange = 4;
        DEBUG_PRINT(F("8.192 "));
        break;

    case 0b10:
        Acc_Resolution = 4096;
        AccRange = 8;
        DEBUG_PRINT(F("4.096 "));
        break;

    case 0b11:
        Acc_Resolution = 2048;
        AccRange = 16;
        DEBUG_PRINT(F("2.048 "));
        break;

    default:
        DEBUG_PRINTLN(F("Accel Scaling Faktör girdisi yanlis..."));
        return MPU_Error;
    }
    DEBUG_PRINTLN(F("Accel Scaling Faktör olarak ayarlandi..."));
    return MPU_Success;
}

void CalibrationAccGyro(struct MPU_REGISTERS *settings){
    AccGyroCalibRegister();
    AccGyroDataCollection();
    writeAccOffsets();
    writeGyroOffsets();
    delay(100);
    mpuInit(settings);
    delay(1000);
}

void AccGyroCalibRegister(){
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, 0x00);
    delay(100);

    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, 0x01);
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT_2, 0x00);
    delay(200);

    I2CWriteByte(MPU_CHIPADR, INT_ENABLE, 0x00);
    I2CWriteByte(MPU_CHIPADR, FIFOENABLE_REG, 0x00);
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, 0x00);
    I2CWriteByte(MPU_CHIPADR, I2C_MASTER_CONTROL, 0x00);
    I2CWriteByte(MPU_CHIPADR, USER_CTRL, 0x00);
    I2CWriteByte(MPU_CHIPADR, USER_CTRL, 0x0C);
    delay(15);

    I2CWriteByte(MPU_CHIPADR, CONFIG, 0x01);
    I2CWriteByte(MPU_CHIPADR, SAMPLERATE_DIVIDER, 0x00);
    I2CWriteByte(MPU_CHIPADR, CONFIG_GYRO, 0x00);
    I2CWriteByte(MPU_CHIPADR, CONFIG_ACCEL, 0x00);

    I2CWriteByte(MPU_CHIPADR, USER_CTRL, 0x40);
    I2CWriteByte(MPU_CHIPADR, FIFOENABLE_REG, 0x78);
    delay(40);
}

void AccGyroDataCollection(){
    uint8_t data[12];
    I2CWriteByte(MPU_CHIPADR, FIFOENABLE_REG, 0x00);
    I2CReadBytes(MPU_CHIPADR, FIFO_COUNTH, data, 2, TIMEOUT_I2C);

    uint16_t fifoCount = ((uint16_t)data[0] << 8) | data[1];
    uint16_t packetCount = fifoCount / 12;

    for(uint16_t i = 0; i < packetCount; i++){
        int16_t accelTemp[3] = {0, 0, 0};
        int16_t gyroTemp[3] = {0, 0, 0};

        I2CReadBytes(MPU_CHIPADR, FIFO_R_W, data, 12, TIMEOUT_I2C);

        accelTemp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
        accelTemp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accelTemp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);

        gyroTemp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyroTemp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyroTemp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        Acc_bias[0] += (float)accelTemp[0];
        Acc_bias[1] += (float)accelTemp[1];
        Acc_bias[2] += (float)accelTemp[2];

        Gyro_bias[0] += (float)gyroTemp[0];
        Gyro_bias[1] += (float)gyroTemp[1];
        Gyro_bias[2] += (float)gyroTemp[2];
    }

    Acc_bias[0] /= (float)packetCount;
    Acc_bias[1] /= (float)packetCount;
    Acc_bias[2] /= (float)packetCount;
    Gyro_bias[0] /= (float)packetCount;
    Gyro_bias[1] /= (float)packetCount;
    Gyro_bias[2] /= (float)packetCount;

    if(Acc_bias[2] > 0L){
        Acc_bias[2] -= (float)CALIB_ACCEL_SENSIVITY;
    }
    else{
        Acc_bias[2] += (float)CALIB_ACCEL_SENSIVITY;
    }
}

// Acc Offset Write
void writeAccOffsets(){
    uint8_t readData[2]{0};
    int16_t accBiasFactory[3] = {0, 0, 0};
    I2CReadBytes(MPU_CHIPADR, ACCEL_X_OFFSET_MSB, readData, 2, TIMEOUT_I2C);
    accBiasFactory[0] = ((int16_t)readData[0] << 8) | readData[1];
    I2CReadBytes(MPU_CHIPADR, ACCEL_Y_OFFSET_MSB, readData, 2, TIMEOUT_I2C);
    accBiasFactory[1] = ((int16_t)readData[0] << 8) | readData[1];
    I2CReadBytes(MPU_CHIPADR, ACCEL_Z_OFFSET_MSB, readData, 2, TIMEOUT_I2C);
    accBiasFactory[2] = ((int16_t)readData[0] << 8) | readData[1];

    int16_t maskBit[3] = {1, 1, 1};
    for(int i = 0; i < 3; i++){
        if(accBiasFactory[i] % 2){
            maskBit[i] = 0;
        }

        accBiasFactory[i] -= (int16_t)Acc_bias[i] >> 3;

        if(maskBit[i]){
            accBiasFactory[i] = accBiasFactory[i] & ~maskBit[i];
        }
        else{
            accBiasFactory[i] = accBiasFactory[i] | 0x0001;
        }
    }

    uint8_t dataWrite[6] = {0};
    dataWrite[0] = (accBiasFactory[0] >> 8) & 0xFF;
    dataWrite[1] = (accBiasFactory[0]) & 0xFF;
    dataWrite[2] = (accBiasFactory[1] >> 8) & 0xFF;
    dataWrite[3] = (accBiasFactory[1]) & 0xFF;
    dataWrite[4] = (accBiasFactory[2] >> 8) & 0xFF;
    dataWrite[5] = (accBiasFactory[2]) & 0xFF;

    I2CWriteByte(MPU_CHIPADR, ACCEL_X_OFFSET_MSB, dataWrite[0]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_X_OFFSET_LSB, dataWrite[1]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_Y_OFFSET_MSB, dataWrite[2]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_Y_OFFSET_LSB, dataWrite[3]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_Z_OFFSET_MSB, dataWrite[4]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_Z_OFFSET_LSB, dataWrite[5]);
}

// Gyro Offset Write
void writeGyroOffsets(){
    uint8_t gyroOffsetData[6] {0};
    gyroOffsetData[0] = (-(int16_t)Gyro_bias[0] / 4 >> 8) & 0xFF;
    gyroOffsetData[1] = (-(int16_t)Gyro_bias[0] / 4) & 0xFF;
    gyroOffsetData[2] = (-(int16_t)Gyro_bias[1] / 4 >> 8) & 0xFF;
    gyroOffsetData[3] = (-(int16_t)Gyro_bias[1] / 4) & 0xFF;
    gyroOffsetData[4] = (-(int16_t)Gyro_bias[2] / 4 >> 8) & 0xFF;
    gyroOffsetData[5] = (-(int16_t)Gyro_bias[2] / 4) & 0xFF;

    I2CWriteByte(MPU_CHIPADR, GYRO_X_OFFSET_MSB, gyroOffsetData[0]);
    I2CWriteByte(MPU_CHIPADR, GYRO_X_OFFSET_LSB, gyroOffsetData[1]);
    I2CWriteByte(MPU_CHIPADR, GYRO_Y_OFFSET_MSB, gyroOffsetData[2]);
    I2CWriteByte(MPU_CHIPADR, GYRO_Y_OFFSET_LSB, gyroOffsetData[3]);
    I2CWriteByte(MPU_CHIPADR, GYRO_Z_OFFSET_MSB, gyroOffsetData[4]);
    I2CWriteByte(MPU_CHIPADR, GYRO_Z_OFFSET_LSB, gyroOffsetData[5]);
}