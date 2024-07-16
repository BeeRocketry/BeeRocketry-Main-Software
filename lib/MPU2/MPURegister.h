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
Status magSetControlRegister(bool continuousmode, uint16_t datarate){
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

    return MPU_Success;
}