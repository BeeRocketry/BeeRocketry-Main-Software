#ifndef ICM20948_H
#define ICM20948_H

#include <Arduino.h>
#include "I2C.h"
#include "debugprinter.h"

#define ICM_CHIPADR 0x68

#define REG_BANK_SELECT 0x7F

// USER BANK 0
#define ICM_REG_WHOAMI_REG 0x00
#define ICM_REG_WHOAMI_VALUE 0xEA
#define ICM_REG_USER_CTRL 0x03
#define ICM_REG_LP_CONFIG 0x05
#define ICM_REG_POWER_MANAGEMENT 0x06
#define ICM_REG_POWER_MANAGEMENT_2 0x07
#define ICM_REG_INT_PIN_CFG 0x0F
#define ICM_REG_INT_ENABLE 0x10
#define ICM_REG_INT_ENABLE_1 0x11
#define ICM_REG_INT_ENABLE_2 0x12
#define ICM_REG_INT_ENABLE_3 0x13
#define ICM_REG_I2C_MASTER_STATUS 0x17
#define ICM_REG_DELAY_TIME_HIGH 0x28
#define ICM_REG_DELAY_TIME_LOW 0x29

#define ICM_REG_ACCEL_X_OUTPUT_MSB 0x2D
#define ICM_REG_GYRO_X_OUTPUT_MSB 0x33
#define ICM_REG_TEMP_OUTPUT_MSB 0x39

#define ICM_REG_FIFO_ENABLE_1 0x66
#define ICM_REG_FIFO_ENABLE_2 0x67

#define ICM_REG_FIFO_MODE 0x69
#define ICM_REG_FIFO_CONFIG 0x76


// USER BANK 2

#define ICM_REG_GYRO_SAMPLERATE 0x00
#define ICM_REG_GYRO_CONFIG_1 0x01
#define ICM_REG_GYRO_CONFIG_2 0x02
#define ICM_REG_ODR_ALIGN 0x09
#define ICM_REG_ACC_SAMPLERATE 0x10
#define ICM_REG_ACC_SAMPLERATE_2 0x11
#define ICM_REG_ACC_INTEL_CTRL 0x12
#define ICM_REG_ACC_WOM 0x13
#define ICM_REG_ACC_CONFIG_1 0x14
#define ICM_REG_ACC_CONFIG_2 0x15
#define ICM_REG_FSYNC_CONFIG 0x52


// USER BANK 3

#define ICM_REG_I2C_MASTER_CTRL 0x01

// USER BANK 0 Enums

typedef enum : uint8_t{
    GENERAL_ON = 0b1,
    GENERAL_OFF = 0b0
}ICM_GENERALONOFF;

typedef enum : uint8_t{
    GENERAL_INVERTED_OFF = 0b1,
    GENERAL_INVERTED_ON = 0b0
}ICM_INVERTED_GENERALONOFF;

typedef enum : uint8_t{
    ICM_CLOCK_Internal20 = 0b000,
    ICM_CLOCK_Gyro = 0b001
}ICM_CLOCKSOURCE;

typedef enum : uint8_t{
    ICM_PWR_2_Disable = 0b111,
    ICM_PWR_2_Enable = 0b000
}ICM_PWR_2_Select;

typedef enum : uint8_t{
    ICM_FIFO_Stream = 0b0,
    ICM_FIFO_Snapshot = 0b1
}ICM_FIFO_MODE_Select;

// USER BANK 0 Structs

struct ICM_USERCONTROL{
    ICM_GENERALONOFF dmpEnable = GENERAL_OFF; // 7. Bit
    ICM_GENERALONOFF fifoEnable = GENERAL_OFF; // 6. Bit
    ICM_GENERALONOFF I2CMasterEnable = GENERAL_OFF; // 5. Bit
};

struct ICM_LPCONFIG{
    ICM_GENERALONOFF I2CMasterCycle = GENERAL_OFF; // 6. Bit
    ICM_GENERALONOFF AccCycle = GENERAL_OFF; // 5. Bit
    ICM_GENERALONOFF GyroCycle = GENERAL_OFF; // 4. Bit
};

struct ICM_PWR_MNGMT_1{
    ICM_GENERALONOFF resetBit = GENERAL_OFF; // 7. Bit
    ICM_GENERALONOFF sleepBit = GENERAL_OFF; // 6. Bit
    ICM_GENERALONOFF LPEnable = GENERAL_OFF; // 5. Bit
    ICM_INVERTED_GENERALONOFF TempEn = GENERAL_INVERTED_OFF; // 3. Bit
    ICM_CLOCKSOURCE clockSelect = ICM_CLOCK_Gyro; // 2-0 Bit
};

struct ICM_PWR_MNGMT_2{
    ICM_PWR_2_Select accDisable = ICM_PWR_2_Enable; // 5-3 Bit
    ICM_PWR_2_Select gyroDisable = ICM_PWR_2_Enable; // 2-0 Bit
};

struct ICM_INT_ENABLE{
    ICM_GENERALONOFF FysncInterrupt = GENERAL_OFF; // 7. Bit
    ICM_GENERALONOFF WakeOnMotionInterrupt = GENERAL_OFF; // 3. Bit
    ICM_GENERALONOFF PLLRDYInterrupt = GENERAL_OFF; // 2. Bit
    ICM_GENERALONOFF DMPInterrupt = GENERAL_OFF; // 1. Bit
    ICM_GENERALONOFF I2CInterrupt = GENERAL_OFF; // 0. Bit
};

struct ICM_INT_ENABLE_1{
    ICM_GENERALONOFF rawDataInterrupt = GENERAL_OFF; // 0. Bit
};

struct ICM_INT_ENABLE_2{
    ICM_GENERALONOFF fifoOverflowInterrupt = GENERAL_OFF; // 4-0 Bit
};

struct ICM_INT_ENABLE_3{
    ICM_GENERALONOFF fifoWatermarkInterrupt = GENERAL_OFF; // 4-0 Bit
};

struct ICM_FIFO_EN_1{
    ICM_GENERALONOFF SLV_3FifoSelect = GENERAL_OFF; // 3. Bit
    ICM_GENERALONOFF SLV_2FifoSelect = GENERAL_OFF; // 2. Bit
    ICM_GENERALONOFF SLV_1FifoSelect = GENERAL_OFF; // 1. Bit
    ICM_GENERALONOFF SLV_0FifoSelect = GENERAL_OFF; // 0. Bit
};

struct ICM_FIFO_EN_2{
    ICM_GENERALONOFF AccFifoSelect = GENERAL_OFF; // 4. Bit
    ICM_GENERALONOFF GyroZFifoSelect = GENERAL_OFF; // 3. Bit
    ICM_GENERALONOFF GyroYFifoSelect = GENERAL_OFF; // 2. Bit
    ICM_GENERALONOFF GyroXFifoSelect = GENERAL_OFF; // 1. Bit
    ICM_GENERALONOFF TempFifoSelect = GENERAL_OFF; // 0. Bit
};

struct ICM_FIFO_MODE{
    ICM_FIFO_MODE_Select fifoModeSel = ICM_FIFO_Stream; // 4-0 Bit
};

// USER BANK 2 Enums

typedef enum : uint8_t{
    ICM_Gyro_Scale_250 = 0b00,
    ICM_Gyro_Scale_500 = 0b01,
    ICM_Gyro_Scale_1000 = 0b10,
    ICM_Gyro_Scale_2000 = 0b11
}ICM_GYRO_SCALE;

typedef enum : uint8_t{
    ICM_WOM_FirstSample = 0b0,
    ICM_WOM_PreviousSample = 0b1
}ICM_WOM_ALGORITHM_SELECT;

typedef enum : uint8_t{
    ICM_Acc_Scale_2g = 0b00,
    ICM_Acc_Scale_4g = 0b01,
    ICM_Acc_Scale_8g = 0b10,
    ICM_Acc_Scale_16g = 0b11
}ICM_ACC_SCALE;

// USER BANK 2 Structs

/*!!!!!!!!!!       GYRO VE ACCEL SAMPLE RATE UNUTMA         !!!!!!!!!!!!!*/

struct ICM_GYRO_CONFIG_1{
    uint8_t GyroDLPFConfig = 3; // 5-3 Bit -- 0 ve 8 arasında
    ICM_GYRO_SCALE gyroFullScale = ICM_Gyro_Scale_1000; // 2-1 Bit
    ICM_GENERALONOFF gyroFChoice = GENERAL_ON; // 0. Bit
};

struct ICM_ODR_ALIGNMENT{
    ICM_GENERALONOFF ODRAlignment = GENERAL_ON; // 0. Bit
};

struct ICM_ACCEL_INTEL_CTRL{
    ICM_GENERALONOFF WakeOnMotionSelect = GENERAL_OFF; // 1. Bit
    ICM_WOM_ALGORITHM_SELECT WakeOnMotionMode = ICM_WOM_FirstSample; // 0. Bit
};

struct ICM_ACCEL_CONFIG_1{
    uint8_t AccDLPFConfig = 3; // 5-3 Bit -- 0 ve 8 arasında
    ICM_ACC_SCALE accFullScale = ICM_Acc_Scale_16g; // 2-1 Bit
    ICM_GENERALONOFF accFChoice = GENERAL_ON; // 0. Bit
};


// User Bank Structs

struct ICM_USERBANK_0{
    ICM_USERCONTROL userCTRL_register; // 0x03
    ICM_LPCONFIG lpower_register; // 0x05
    ICM_PWR_MNGMT_1 powerMNGMNT_1_register; // 0x06
    ICM_PWR_MNGMT_2 powerMNGMNT_2_register; // 0x07
    ICM_INT_ENABLE interruptEnable_register; // 0x10
    ICM_INT_ENABLE_1 interruptEnable_1_register; // 0x11
    ICM_INT_ENABLE_2 interruptEnable_2_register; // 0x12
    ICM_INT_ENABLE_3 interruptEnable_3_register; // 0x13
    ICM_FIFO_EN_1 fifoEnable_Slave_register; // 0x66
    ICM_FIFO_EN_2 fifoEnable_Master_register; // 0x67
    ICM_FIFO_MODE fifoMode_register; // 0x69
};

struct ICM_USERBANK_2{
    ICM_GYRO_CONFIG_1 gyroConfig_register; // 0x01
    ICM_ODR_ALIGNMENT odrAlignment_register; // 0x09
    ICM_ACCEL_INTEL_CTRL accIntelCtrl_register; // 0x12
    ICM_ACCEL_CONFIG_1 accConfig_register; // 0x14
    uint8_t gyroSampleRate_Value; // 0x00
    uint16_t accSampleRate_Value; // 11-8 0x10 7-0 0x11
};

// ICM General Struct

struct ICM_REGISTER{
    ICM_USERBANK_0 userBank0_register;
    ICM_USERBANK_2 userBank2_register;
};

struct ICM_DOF3_FLOAT{
    float x;
    float y;
    float z;
};

struct ICM_DOF3_INT16{
    int16_t x;
    int16_t y;
    int16_t z;
};

void ICMBegin(ICM_REGISTER *registers);
void ICM_setGyroResolution(ICM_REGISTER *registers);
void ICM_setAccResolution(ICM_REGISTER *registers);
void ICM_setResolutions(ICM_REGISTER *registers);
void ICM_setUserBank(uint8_t Bank);
uint8_t ICM_getUserBank();
void ICM_setUserBank0(ICM_REGISTER *registers);
void ICM_setUserBank2(ICM_REGISTER *registers);
void ICM_setUserControlReg(ICM_REGISTER *registers);
void ICM_setLPConfigReg(ICM_REGISTER *registers);
void ICM_setPowerManagement1Reg(ICM_REGISTER *registers);
void ICM_setPowerManagement2Reg(ICM_REGISTER *registers);
void ICM_setIntEnable0Reg(ICM_REGISTER *registers);
void ICM_setIntEnable1Reg(ICM_REGISTER *registers);
void ICM_setIntEnable2Reg(ICM_REGISTER *registers);
void ICM_setIntEnable3Reg(ICM_REGISTER *registers);
void ICM_setFIFOEnable1Reg(ICM_REGISTER *registers);
void ICM_setFIFOEnable2Reg(ICM_REGISTER *registers);
void ICM_setFIFOModeReg(ICM_REGISTER *registers);
void ICM_setGyroConfigReg(ICM_REGISTER *registers);
void ICM_setODRAlignmentReg(ICM_REGISTER *registers);
void ICM_setAccIntelReg(ICM_REGISTER *registers);
void ICM_setAccConfigReg(ICM_REGISTER *registers);
void ICM_setGyroSampleRate(ICM_REGISTER *registers);
void ICM_setAccSampleRate(ICM_REGISTER *registers);
void ICM_getRawAccData(ICM_DOF3_INT16 *AccData);
void ICM_getRawGyroData(ICM_DOF3_INT16 *GyroData);
void ICM_getNormalizedAccData(ICM_DOF3_INT16 *RawData, ICM_DOF3_FLOAT *AccData);
void ICM_getNormalizedGyroData(ICM_DOF3_INT16 *RawData, ICM_DOF3_FLOAT *GyroData);
void ICM_getRawAccGyroData(ICM_DOF3_INT16 *AccRaw, ICM_DOF3_INT16 *GyroRaw);
void ICM_GyroCalibration(uint32_t numSample);
void ICM_setReset();
void ICM_setSleepMode(bool sleep);


#endif