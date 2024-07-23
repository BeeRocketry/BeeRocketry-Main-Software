#include "ICM20948.h"

float Acc_Resolution = 2048;
float Gyro_Resolution = 328;
int16_t AccOffsets[3] = {0};
int16_t GyroOffsets[3] = {0};

void ICMBegin(ICM_REGISTER *registers){
    ICM_setResolutions(registers);
    delay(10);

    ICM_setReset();
    delay(20);

    ICM_setUserBank(2);
    delay(10);

    ICM_setUserBank2(registers);

    ICM_setUserBank(0);
    delay(10);

    ICM_setUserBank0(registers);
    delay(10);
}

void ICM_setGyroResolution(ICM_REGISTER *registers){
    switch (registers->userBank2_register.gyroConfig_register.gyroFullScale)
    {
    case ICM_Gyro_Scale_250:
        Gyro_Resolution = 131;
        break;

    case ICM_Gyro_Scale_500:
        Gyro_Resolution = 65.5;
        break;

    case ICM_Gyro_Scale_1000:
        Gyro_Resolution = 32.8;
        break;

    case ICM_Gyro_Scale_2000:
        Gyro_Resolution = 16.4;
        break;
    
    default:
        Gyro_Resolution = 32.8;
        break;
    }
}

void ICM_setAccResolution(ICM_REGISTER *registers){
    switch (registers->userBank2_register.accConfig_register.accFullScale)
    {
    case ICM_Acc_Scale_2g:
        Acc_Resolution = 16384;
        break;

    case ICM_Acc_Scale_4g:
        Acc_Resolution = 8192;
        break;

    case ICM_Acc_Scale_8g:
        Acc_Resolution = 4096;
        break;

    case ICM_Acc_Scale_16g:
        Acc_Resolution = 2048;
        break;
    
    default:
        break;
    }
}

void ICM_setResolutions(ICM_REGISTER *registers){
    ICM_setGyroResolution(registers);
    ICM_setAccResolution(registers);
}

void ICM_setUserBank(uint8_t Bank){
    uint8_t reg = 0;

    reg |= (Bank << 4);

    I2CWriteByte(ICM_CHIPADR, REG_BANK_SELECT, reg);
    delay(5);
}

uint8_t ICM_getUserBank(){
    uint8_t reg = 0;

    I2CReadByte(ICM_CHIPADR, REG_BANK_SELECT, &reg, TIMEOUT_I2C);

    reg = reg >> 4;

    return reg;
}

void ICM_setUserBank0(ICM_REGISTER *registers){
    if(ICM_getUserBank() != 0){
        ICM_setUserBank(0);
        delay(5);
    }

    ICM_setUserControlReg(registers);
    delay(10);

    ICM_setLPConfigReg(registers);
    delay(10);

    ICM_setPowerManagement2Reg(registers);
    delay(10);

    ICM_setIntEnable0Reg(registers);
    delay(10);

    ICM_setIntEnable1Reg(registers);
    delay(10);
    
    ICM_setIntEnable2Reg(registers);
    delay(10);

    ICM_setIntEnable3Reg(registers);
    delay(10);

    ICM_setFIFOEnable1Reg(registers);
    delay(10);

    ICM_setFIFOEnable2Reg(registers);
    delay(10);

    ICM_setFIFOModeReg(registers);
    delay(10);
}

void ICM_setUserBank2(ICM_REGISTER *registers){
    if(ICM_getUserBank() != 2){
        ICM_setUserBank(2);
        delay(5);
    }

    ICM_setGyroConfigReg(registers);
    delay(10);

    ICM_setODRAlignmentReg(registers);
    delay(10);

    ICM_setAccIntelReg(registers);
    delay(10);

    ICM_setAccConfigReg(registers);
    delay(10);

    ICM_setGyroSampleRate(registers);
    delay(10);

    ICM_setAccSampleRate(registers);
    delay(10);
}

void ICM_setUserControlReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_USERCONTROL tempStruct;
    tempStruct = registers->userBank0_register.userCTRL_register;

    reg |= (tempStruct.dmpEnable << 7) | (tempStruct.fifoEnable << 6) | (tempStruct.I2CMasterEnable << 5);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_USER_CTRL, reg);
}

void ICM_setLPConfigReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_LPCONFIG tempStruct;
    tempStruct = registers->userBank0_register.lpower_register;

    reg |= (tempStruct.I2CMasterCycle << 6) | (tempStruct.AccCycle << 5) | (tempStruct.GyroCycle << 4);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_LP_CONFIG, reg);
}

void ICM_setPowerManagement1Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_PWR_MNGMT_1 tempStruct;
    tempStruct = registers->userBank0_register.powerMNGMNT_1_register;

    reg |= (tempStruct.resetBit << 7) | (tempStruct.sleepBit << 6) | (tempStruct.LPEnable << 5) | (tempStruct.TempEn << 3) | (tempStruct.clockSelect);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_POWER_MANAGEMENT, reg);
}

void ICM_setPowerManagement2Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_PWR_MNGMT_2 tempStruct;
    tempStruct = registers->userBank0_register.powerMNGMNT_2_register;

    reg |= (tempStruct.accDisable << 3) | (tempStruct.gyroDisable);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_POWER_MANAGEMENT_2, reg);
}

void ICM_setIntEnable0Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_INT_ENABLE tempStruct;
    tempStruct = registers->userBank0_register.interruptEnable_register;

    reg |= (tempStruct.FysncInterrupt << 7) | (tempStruct.WakeOnMotionInterrupt << 3) | (tempStruct.PLLRDYInterrupt << 2) | (tempStruct.DMPInterrupt << 1) | (tempStruct.I2CInterrupt);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_INT_ENABLE, reg);
}

void ICM_setIntEnable1Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_INT_ENABLE_1 tempStruct;
    tempStruct = registers->userBank0_register.interruptEnable_1_register;

    reg |= (tempStruct.rawDataInterrupt);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_INT_ENABLE_1, reg);
}

void ICM_setIntEnable2Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_INT_ENABLE_2 tempStruct;
    tempStruct = registers->userBank0_register.interruptEnable_2_register;

    reg |= (tempStruct.fifoOverflowInterrupt);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_INT_ENABLE_2, reg);
}

void ICM_setIntEnable3Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_INT_ENABLE_3 tempStruct;
    tempStruct = registers->userBank0_register.interruptEnable_3_register;

    reg |= (tempStruct.fifoWatermarkInterrupt);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_INT_ENABLE_3, reg);
}

void ICM_setFIFOEnable1Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_FIFO_EN_1 tempStruct;
    tempStruct = registers->userBank0_register.fifoEnable_Slave_register;

    reg |= (tempStruct.SLV_3FifoSelect << 3) | (tempStruct.SLV_2FifoSelect << 2) | (tempStruct.SLV_1FifoSelect << 1) | (tempStruct.SLV_0FifoSelect);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_FIFO_ENABLE_1, reg);
}

void ICM_setFIFOEnable2Reg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_FIFO_EN_2 tempStruct;
    tempStruct = registers->userBank0_register.fifoEnable_Master_register;

    reg |= (tempStruct.AccFifoSelect << 4) | (tempStruct.GyroZFifoSelect << 3) | (tempStruct.GyroYFifoSelect << 2) | (tempStruct.GyroXFifoSelect << 1) | (tempStruct.TempFifoSelect);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_FIFO_ENABLE_2, reg);
}

void ICM_setFIFOModeReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_FIFO_MODE tempStruct;
    tempStruct = registers->userBank0_register.fifoMode_register;

    reg |= (tempStruct.fifoModeSel);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_FIFO_MODE, reg);
}

void ICM_setGyroConfigReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_GYRO_CONFIG_1 tempStruct;
    tempStruct = registers->userBank2_register.gyroConfig_register;

    reg |= (tempStruct.GyroDLPFConfig << 3) | (tempStruct.gyroFullScale << 1) | (tempStruct.gyroFChoice);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_GYRO_CONFIG_1, reg);
}

void ICM_setODRAlignmentReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_ODR_ALIGNMENT tempStruct;
    tempStruct = registers->userBank2_register.odrAlignment_register;

    reg |= (tempStruct.ODRAlignment);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_ODR_ALIGN, reg);
}

void ICM_setAccIntelReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_ACCEL_INTEL_CTRL tempStruct;
    tempStruct = registers->userBank2_register.accIntelCtrl_register;

    reg |= (tempStruct.WakeOnMotionSelect << 1) | (tempStruct.WakeOnMotionMode);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_ACC_INTEL_CTRL, reg);
}

void ICM_setAccConfigReg(ICM_REGISTER *registers){
    uint8_t reg = 0;
    ICM_ACCEL_CONFIG_1 tempStruct;
    tempStruct = registers->userBank2_register.accConfig_register;

    reg |= (tempStruct.AccDLPFConfig << 3) | (tempStruct.accFullScale << 1) | (tempStruct.accFChoice);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_ACC_CONFIG_1, reg);
}

void ICM_setGyroSampleRate(ICM_REGISTER *registers){
    uint8_t reg = 0;
    reg = registers->userBank2_register.gyroSampleRate_Value;

    I2CWriteByte(ICM_CHIPADR, ICM_REG_GYRO_SAMPLERATE, reg);
}

void ICM_setAccSampleRate(ICM_REGISTER *registers){
    union int16to8{
        int16_t mainRate;
        int8_t splittedRate[2];
    }ConversionforAccSample;
    
    ConversionforAccSample.mainRate = registers->userBank2_register.accSampleRate_Value;

    uint8_t reg = 0;
    reg = ConversionforAccSample.splittedRate[1];

    I2CWriteByte(ICM_CHIPADR, ICM_REG_ACC_SAMPLERATE, reg);
    delay(10);

    reg = ConversionforAccSample.splittedRate[0];

    I2CWriteByte(ICM_CHIPADR, ICM_REG_ACC_SAMPLERATE_2, reg);
}

void ICM_setSleepMode(bool sleep){
    uint8_t reg = 0;
    I2CReadByte(ICM_CHIPADR, ICM_REG_POWER_MANAGEMENT, &reg, TIMEOUT_I2C);

    if(sleep == true){
        reg |= (sleep << 6);
    }
    else{
        reg &= (sleep << 6);
    }

    I2CWriteByte(ICM_CHIPADR, ICM_REG_POWER_MANAGEMENT, reg);
}

void ICM_setReset(){
    uint8_t reg = 0;
    I2CReadByte(ICM_CHIPADR, ICM_REG_POWER_MANAGEMENT, &reg, TIMEOUT_I2C);

    reg |= ((uint8_t)0b1 << 7);

    I2CWriteByte(ICM_CHIPADR, ICM_REG_POWER_MANAGEMENT, reg);
}

void ICM_getRawAccData(ICM_DOF3_INT16 *AccData){
    if(ICM_getUserBank != 0){
        ICM_setUserBank(0);
        delay(5);
    }

    uint8_t bufferAcc[6];

    I2CReadBytes(ICM_CHIPADR, ICM_REG_ACCEL_X_OUTPUT_MSB, bufferAcc, 6, TIMEOUT_I2C);

    AccData->x = ((uint16_t)bufferAcc[0] << 8) | (bufferAcc[1]);
    AccData->y = ((uint16_t)bufferAcc[2] << 8) | (bufferAcc[3]);
    AccData->z = ((uint16_t)bufferAcc[4] << 8) | (bufferAcc[5]);
}

void ICM_getRawGyroData(ICM_DOF3_INT16 *GyroData){
    if(ICM_getUserBank != 0){
        ICM_setUserBank(0);
        delay(5);
    }

    uint8_t bufferAcc[6];

    I2CReadBytes(ICM_CHIPADR, ICM_REG_GYRO_X_OUTPUT_MSB, bufferAcc, 6, TIMEOUT_I2C);

    GyroData->x = ((uint16_t)bufferAcc[0] << 8) | (bufferAcc[1]);
    GyroData->y = ((uint16_t)bufferAcc[2] << 8) | (bufferAcc[3]);
    GyroData->z = ((uint16_t)bufferAcc[4] << 8) | (bufferAcc[5]);
}

void ICM_getNormalizedAccData(ICM_DOF3_INT16 *RawData, ICM_DOF3_FLOAT *AccData){
    AccData->x = (RawData->x - AccOffsets[0]) / Acc_Resolution;
    AccData->y = (RawData->y - AccOffsets[1]) / Acc_Resolution;
    AccData->z = (RawData->z - AccOffsets[2]) / Acc_Resolution;
}

void ICM_getNormalizedGyroData(ICM_DOF3_INT16 *RawData, ICM_DOF3_FLOAT *GyroData){
    GyroData->x = (RawData->x - AccOffsets[0]) / Gyro_Resolution;
    GyroData->y = (RawData->y - AccOffsets[1]) / Gyro_Resolution;
    GyroData->z = (RawData->z - AccOffsets[2]) / Gyro_Resolution;
}

void ICM_getRawAccGyroData(ICM_DOF3_INT16 *AccRaw, ICM_DOF3_INT16 *GyroRaw){
    if(ICM_getUserBank != 0){
        ICM_setUserBank(0);
        delay(5);
    }

    uint8_t bufferAcc[12];

    I2CReadBytes(ICM_CHIPADR, ICM_REG_ACCEL_X_OUTPUT_MSB, bufferAcc, 12, TIMEOUT_I2C);

    AccRaw->x = ((uint16_t)bufferAcc[0] << 8) | (bufferAcc[1]);
    AccRaw->y = ((uint16_t)bufferAcc[2] << 8) | (bufferAcc[3]);
    AccRaw->z = ((uint16_t)bufferAcc[4] << 8) | (bufferAcc[5]);
    GyroRaw->x = ((uint16_t)bufferAcc[6] << 8) | (bufferAcc[7]);
    GyroRaw->y = ((uint16_t)bufferAcc[8] << 8) | (bufferAcc[9]);
    GyroRaw->z = ((uint16_t)bufferAcc[10] << 8) | (bufferAcc[11]);
}

void ICM_GyroCalibration(uint32_t numSample){
    ICM_DOF3_INT16 gyro;
    int gyromin[3] = {32767 , 32767 , 32767};
    int gyromax[3] = {-32767 , -32767 , -32767};
    int gyromean[3];
    for(int i = 0; i < numSample; i++){
        if(numSample % 100 == 0){
            DEBUG_PRINT(F("Sample "));
            DEBUG_PRINTLN(i);
        }

        ICM_getRawGyroData(&gyro);

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

    while(1){

    }
}