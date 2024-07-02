#include "MPU.h"

// Power Management Register Ayarlamasını Yapar
Status mpuSetPowerMngmtRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_PowerManagament1_Register.activationmode << 6));
    reg = (reg | ((uint8_t)settings->MPU_PowerManagament1_Register.clockSet));

    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, reg);

    return MPU_Success;
}

// Config Register Ayarlamasını Yapar
Status mpuSetConfigRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_Config_Register.fifoMode << 6));
    reg = (reg | ((uint8_t)settings->MPU_Config_Register.syncMode << 3));
    reg = (reg | ((uint8_t)settings->MPU_Config_Register.dlpfMode));

    I2CWriteByte(MPU_CHIPADR, CONFIG, reg);

    return MPU_Success;
}

// Gyro Config Register Ayarlamasını Yapar
Status mpuSetGyroConfigRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_GyroConfig_Register.gyroScale << 3));
    reg = (reg | ((uint8_t)settings->MPU_GyroConfig_Register.dlpfchoice));

    I2CWriteByte(MPU_CHIPADR, CONFIG_GYRO, reg);

    return MPU_Success;
}

// İvme Config Register Ayarlamasini Yapar
Status mpuSetAccConfigRegister(struct MPU_REGISTERS *settings){
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
Status mpuSetI2CMasterRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_I2CMaster_Register.masterClock));

    I2CWriteByte(MPU_CHIPADR, I2C_MASTER_CONTROL, reg);

    return MPU_Success;
}

// Interrupt Pin Register Ayarlamasini Yapar
Status mpuSetIntPinRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_INTPin_Register.i2cbypassMode << 1));

    I2CWriteByte(MPU_CHIPADR, INT_PIN_CFG, reg);

    return MPU_Success;
}

// User Control Register Ayarlamasini Yapar
Status mpuSetUserControlRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MPU_UserControl_Register.fifoEnable << 6));
    reg = (reg | ((uint8_t)settings->MPU_UserControl_Register.i2cMasterEnable << 5));

    I2CWriteByte(MPU_CHIPADR, USER_CTRL, reg);

    DEBUG_PRINTLN(F("MPU USER Control Registeri Ayarlandi..."));

    return MPU_Success;
}

// Magnetometer Control Register Ayarlamasini Yapar
Status magSetControlRegister(struct MPU_REGISTERS *settings){
    uint8_t reg = 0;

    reg = (reg | ((uint8_t)settings->MAG_Control_Register.operationMode));
    reg = (reg | ((uint8_t)settings->MAG_Control_Register.outputBit << 4));

    I2CWriteByte(MAG_CHIPADR, MAG_CTRL, reg);

    DEBUG_PRINTLN(F("MPU Control Registeri Ayarlandi..."));

    return MPU_Success;
}

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