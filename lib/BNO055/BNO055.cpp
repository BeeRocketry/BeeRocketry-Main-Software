#include "BNO055.h"

float BNO_Acc_Resolution = 100;
float BNO_Gyro_Resolution = 16;
float BNO_Mag_Resolution = 16;
int16_t BNO_AccOffsets[3] = {0};
int16_t BNO_GyroOffsets[3] = {0};
int16_t BNO_MagOffsets[3] = {0, 0, 0};

void writeOffsets(BNO_DOF3_int16 accOff, BNO_DOF3_int16 gyroOff, BNO_DOF3_int16 magOff, int16_t accRange, int16_t magRange){
    int16toint8 transformer;

    transformer.int16var = accOff.x;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_OFFSET_X_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_OFFSET_X_MSB, transformer.int8var[1]);

    transformer.int16var = accOff.y;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_OFFSET_Y_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_OFFSET_Y_MSB, transformer.int8var[1]);

    transformer.int16var = accOff.z;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_OFFSET_Z_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_OFFSET_Z_MSB, transformer.int8var[1]);

    transformer.int16var = magOff.x;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_OFFSET_X_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_OFFSET_X_MSB, transformer.int8var[1]);

    transformer.int16var = magOff.y;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_OFFSET_Y_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_OFFSET_Y_MSB, transformer.int8var[1]);
    
    transformer.int16var = magOff.z;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_OFFSET_Z_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_OFFSET_Z_MSB, transformer.int8var[1]);

    transformer.int16var = gyroOff.x;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_OFFSET_X_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_OFFSET_X_MSB, transformer.int8var[1]);

    transformer.int16var = gyroOff.y;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_OFFSET_Y_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_OFFSET_Y_MSB, transformer.int8var[1]);
    
    transformer.int16var = gyroOff.z;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_OFFSET_Z_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_OFFSET_Z_MSB, transformer.int8var[1]);

    transformer.int16var = accRange;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_RADIUS_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_RADIUS_MSB, transformer.int8var[1]);

    transformer.int16var = magRange;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_RADIUS_LSB, transformer.int8var[0]);
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_RADIUS_MSB, transformer.int8var[1]);
}

void BNOBegin(BNO_STR_REGISTERS registersSet){
    BNO_DOF3_int16 accSystemData, gyroSystemData, magSystemData;
    int16_t accRadius, magRadius;
    accSystemData.x = -4;
    accSystemData.y = 5;
    accSystemData.z = -32;

    magSystemData.x = -490;
    magSystemData.y = -452;
    magSystemData.z = 205;

    gyroSystemData.x = -1;
    gyroSystemData.y = -2;
    gyroSystemData.z = 2;

    accRadius = 1000;
    magRadius = 813;

    changePage(0x00);
    BNO_OPERATIONMODE tempMode = registersSet.registersPage_0.operationMode.operationMode;
    registersSet.registersPage_0.operationMode.operationMode = OPERATIONMODE_CONFIG;
    setOperationMode(registersSet);
    delay(10);
    setUnits(registersSet);
    delay(10);
    setPowerMode(registersSet);
    delay(10);
    //writeOffsets(accSystemData, gyroSystemData, magSystemData, accRadius, magRadius);

    delay(20);

    changePage(0x01);
    setAccConfig(registersSet);
    delay(10);
    setGyroConfig(registersSet);
    delay(10);
    setMagConfig(registersSet);
    delay(10);
    setIntEnable(registersSet);
    delay(10);
    setAccIntSettings(registersSet);
    delay(10);

    registersSet.registersPage_0.operationMode.operationMode = tempMode;

    changePage(0x00);
    setOperationMode(registersSet);
    delay(20);
}

void setOperationMode(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;

    reg |= registersSet.registersPage_0.operationMode.operationMode;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_OPERATIONMODE, reg);
}

BNO_OPERATIONMODE getOperationMode(void){
    uint8_t reg;

    I2CReadByte(BNO_I2C_Adr, BNO_REG_OPERATIONMODE, &reg, TIMEOUT_I2C);

    return (BNO_OPERATIONMODE)reg;
}

void setUnits(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;
    BNO_STR_UNITSELECT tempStruct;
    tempStruct = registersSet.registersPage_0.unitSelection;

    reg |= (tempStruct.dataUnit << 7) | (tempStruct.tempUnit << 4) | (tempStruct.eulerUnit << 2) | (tempStruct.gyroUnit << 1) | (tempStruct.accUnit);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_UNITSELECT, reg);
}

void setPowerMode(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;

    reg |= (registersSet.registersPage_0.powerMode.powerMode);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_POWERMODE, reg);
}

void setAccConfig(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;
    BNO_STR_ACCCONFIG tempStruct;
    tempStruct = registersSet.registersPage_1.accConfig;

    reg |= (tempStruct.accOperation << 5) | (tempStruct.accBandwidth << 2) | tempStruct.accScale;

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACCEL_CONFIG, reg);
}

void setGyroConfig(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;
    BNO_STR_GYRO_CONFIG_0 tempStruct_0;
    BNO_STR_GYRO_CONFIG_1 tempStruct_1;
    tempStruct_0 = registersSet.registersPage_1.gyroConfig;
    tempStruct_1 = registersSet.registersPage_1.gyroConfig_1;

    reg |= (tempStruct_0.gyroBandwidth << 3) | (tempStruct_0.gyroScale);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_CONFIG_0, reg);

    reg = 0;
    
    delay(5);

    reg |= (tempStruct_1.gyroOperation);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_GYRO_CONFIG_1, reg);
}

void setMagConfig(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;
    BNO_STR_MAG_CONFIG tempStruct;
    tempStruct = registersSet.registersPage_1.magConfig;

    reg |= (tempStruct.magPower << 5) | (tempStruct.magOperation << 3) | (tempStruct.magOutputRate);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_MAG_CONFIG, reg);
}

void setIntEnable(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;
    BNO_STR_INT_EN tempStruct;
    tempStruct = registersSet.registersPage_1.intEnable;

    reg |= (tempStruct.accNoMotion << 7) | (tempStruct.accAnyMotion << 6) | (tempStruct.accHighG << 5)
           | (tempStruct.gyroDataReady << 4) | (tempStruct.gyroHighRate << 3) | (tempStruct.gyroAnyMotion << 2)
           | (tempStruct.magDataReady << 1) | (tempStruct.accDataReady);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_INT_EN, reg);
}

void setAccIntSettings(BNO_STR_REGISTERS registersSet){
    uint8_t reg = 0;
    BNO_STR_ACC_INT_SETTINGS tempStruct;
    tempStruct = registersSet.registersPage_1.accIntSettings;

    reg |= (tempStruct.HighG_X << 7) | (tempStruct.HighG_Y << 6) | (tempStruct.HighG_Z << 5)
            | (tempStruct.AnyMotion_X << 4) | (tempStruct.AnyMotion_Y << 3) | (tempStruct.AnyMotion_Z << 2)
            | (tempStruct.AnyMotionDuration);

    I2CWriteByte(BNO_I2C_Adr, BNO_REG_ACC_INT_SETTINGS, reg);
}

void BnoReset(){
    uint8_t reg = 0;
    
    reg |= 0b00100000;
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_SYSTRIGGER, reg);
    delay(10);
}

void BnoSetExternalClock(){
    uint8_t reg = 0;

    reg |= 0b10000000;
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_SYSTRIGGER, reg);
}

void changePage(uint8_t page){
    I2CWriteByte(BNO_I2C_Adr, BNO_REG_PAGE_CHANGE, page);
}

void getCalibrationStatus(uint8_t *systemCalib, uint8_t *gyroCalib, uint8_t *accCalib, uint8_t *magCalib){
    uint8_t reg = 0;

    I2CReadByte(BNO_I2C_Adr, BNO_REG_CALIB_STATUS, &reg, TIMEOUT_I2C);

    *systemCalib = (reg & 0b11000000) >> 6;
    *gyroCalib = (reg & 0b00110000) >> 4;
    *accCalib = (reg & 0b00001100) >> 2;
    *magCalib = (reg & 0b00000011);
}

BNO_DOF3_Float getAccData(){
    BNO_DOF3_int16 accData;
    BNO_DOF3_Float returnData;
    uint8_t buffer[6];

    I2CReadBytes(BNO_I2C_Adr, BNO_REG_ACC_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    accData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    accData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    accData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    returnData.x = (float)accData.x / BNO_Acc_Resolution;
    returnData.y = (float)accData.y / BNO_Acc_Resolution;
    returnData.z = (float)accData.z / BNO_Acc_Resolution;

    return returnData;
}

void getRawGyroData(BNO_DOF3_int16 *gyroData){
    uint8_t buffer[6];

    I2CReadBytes(BNO_I2C_Adr, BNO_REG_GYRO_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    gyroData->x = ((int16_t)buffer[1] << 8) | buffer[0];
    gyroData->y = ((int16_t)buffer[3] << 8) | buffer[2];
    gyroData->z = ((int16_t)buffer[5] << 8) | buffer[4];
}

BNO_DOF3_Float getGyroData(){
    BNO_DOF3_int16 gyroData;
    BNO_DOF3_Float returnData;
    uint8_t buffer[6];

    I2CReadBytes(BNO_I2C_Adr, BNO_REG_GYRO_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    gyroData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    gyroData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    gyroData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    gyroData.x -= BNO_GyroOffsets[0];
    gyroData.y -= BNO_GyroOffsets[1];
    gyroData.z -= BNO_GyroOffsets[2];

    returnData.x = (float)gyroData.x / BNO_Gyro_Resolution;
    returnData.y = (float)gyroData.y / BNO_Gyro_Resolution;
    returnData.z = (float)gyroData.z / BNO_Gyro_Resolution;

    return returnData;
}

BNO_DOF3_Float getMagData(){
    BNO_DOF3_int16 magData;
    BNO_DOF3_Float returnData;
    uint8_t buffer[6];

    I2CReadBytes(BNO_I2C_Adr, BNO_REG_MAG_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    magData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    magData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    magData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    magData.x -= BNO_MagOffsets[0];
    magData.y -= BNO_MagOffsets[1];
    magData.z -= BNO_MagOffsets[2];

    returnData.x = (float)magData.x / BNO_Mag_Resolution;
    returnData.y = (float)magData.y / BNO_Mag_Resolution;
    returnData.z = (float)magData.z / BNO_Mag_Resolution;

    return returnData;
}

void BNO_GyroCalibration(uint32_t numSample){
    BNO_DOF3_int16 gyro;
    int16_t gyromin[3] = {32767 , 32767 , 32767};
    int16_t gyromax[3] = {-32767 , -32767 , -32767};
    for(int i = 0; i < numSample; i++){
        if(numSample % 100 == 0){
            DEBUG_PRINT(F("Sample "));
            DEBUG_PRINTLN(i);
        }

        getRawGyroData(&gyro);

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
        BNO_GyroOffsets[i] = (gyromax[i] + gyromin[i]) / 2;
    }
}