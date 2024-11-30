#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include "I2C.h"

#define BNO_I2C_Adr 0x28

// Register Page 0

#define BNO_REG_CHIPID                  0x00
#define BNO_REG_ACC_ID                  0x01
#define BNO_REG_MAG_ID                  0x02
#define BNO_REG_GYRO_ID                 0x03
#define BNO_REG_ACC_DATA_X_LSB          0x08
#define BNO_REG_MAG_DATA_X_LSB          0x0E
#define BNO_REG_GYRO_DATA_X_LSB         0x14
#define BNO_REG_EULER_H_LSB             0x1A
#define BNO_REG_QUAT_DATA_W_LSB         0x20
#define BNO_REG_LIA_DATA_X_LSB          0x28
#define BNO_REG_GRV_DATA_X_LSB          0x2E
#define BNO_REG_TEMP                    0x34
#define BNO_REG_CALIB_STATUS            0x35
#define BNO_REG_ST_RESULT               0x36
#define BNO_REG_INT_STATUS              0x37
#define BNO_REG_SYS_CLOCK_STATUS        0x38
#define BNO_REG_SYS_STATUS              0x39
#define BNO_REG_SYS_ERROR               0x3A
#define BNO_REG_UNITSELECT              0x3B
#define BNO_REG_OPERATIONMODE           0x3D
#define BNO_REG_POWERMODE               0x3E
#define BNO_REG_SYSTRIGGER              0x3F
#define BNO_REG_TEMPSOURCE              0x40
#define BNO_REG_AXISMAPCONF             0x41
#define BNO_REG_AXISMAPSIGN             0x42
#define BNO_REG_SIC_MATRIX_LSB          0x43
#define BNO_REG_ACC_OFFSET_X_LSB        0x55
#define BNO_REG_ACC_OFFSET_X_MSB        0x56
#define BNO_REG_ACC_OFFSET_Y_LSB        0x57
#define BNO_REG_ACC_OFFSET_Y_MSB        0x58
#define BNO_REG_ACC_OFFSET_Z_LSB        0x59
#define BNO_REG_ACC_OFFSET_Z_MSB        0x5A
#define BNO_REG_MAG_OFFSET_X_LSB        0x5B
#define BNO_REG_MAG_OFFSET_X_MSB        0x5C
#define BNO_REG_MAG_OFFSET_Y_LSB        0x5D
#define BNO_REG_MAG_OFFSET_Y_MSB        0x5E
#define BNO_REG_MAG_OFFSET_Z_LSB        0x5F
#define BNO_REG_MAG_OFFSET_Z_MSB        0x60
#define BNO_REG_GYRO_OFFSET_X_LSB       0x61
#define BNO_REG_GYRO_OFFSET_X_MSB       0x62
#define BNO_REG_GYRO_OFFSET_Y_LSB       0x63
#define BNO_REG_GYRO_OFFSET_Y_MSB       0x64
#define BNO_REG_GYRO_OFFSET_Z_LSB       0x65
#define BNO_REG_GYRO_OFFSET_Z_MSB       0x66
#define BNO_REG_ACC_RADIUS_LSB          0x67
#define BNO_REG_ACC_RADIUS_MSB          0x68
#define BNO_REG_MAG_RADIUS_LSB          0x69
#define BNO_REG_MAG_RADIUS_MSB          0x6A


// Registers Page 1
#define BNO_REG_ACCEL_CONFIG         0x08
#define BNO_REG_MAG_CONFIG           0x09
#define BNO_REG_GYRO_CONFIG_0        0x0A
#define BNO_REG_GYRO_CONFIG_1        0x0B
#define BNO_REG_ACCEL_SLEEP_CONFIG   0x0C
#define BNO_REG_GYRO_SLEEP_CONFIG    0x0D
#define BNO_REG_INT_MASK             0x0F
#define BNO_REG_INT_EN               0x10
#define BNO_REG_ACC_AM_THRES         0x11
#define BNO_REG_ACC_INT_SETTINGS     0x12
#define BNO_REG_ACC_HG_DURATION      0x13
#define BNO_REG_ACC_HG_THRES         0x14
#define BNO_REG_ACC_NM_THRES         0x15
#define BNO_REG_ACC_NM_SET           0x16
#define BNO_REG_GYR_INT_SETTING      0x17
#define BNO_REG_GYR_HR_X_SET         0x18
#define BNO_REG_GYR_DUR_X            0x19
#define BNO_REG_GYR_HR_Y_SET         0x1A
#define BNO_REG_GYR_DUR_Y            0x1B
#define BNO_REG_GYR_HR_Z_SET         0x1C
#define BNO_REG_GYR_DUR_Z            0x1D
#define BNO_REG_GYR_AM_THRES         0x1E
#define BNO_REG_GYR_AM_SET           0x1F

typedef enum : uint8_t{
    BNO_ON = 0b1,
    BNO_OFF = 0b0
}BNO_GENERALONOFF;

typedef enum : uint8_t{
    ACC_UNIT_m2s = 0b0,
    ACC_UNIT_mg = 0b1
}BNO_ACC_UNITSELECT;

typedef enum : uint8_t{
    GYRO_UNIT_dps = 0b0,
    GYRO_UNIT_rps = 0b1
}BNO_GYRO_UNITSELECT;

typedef enum : uint8_t{
    EULER_UNIT_degree = 0b0,
    EULER_UNIT_radian = 0b1
}BNO_EULER_UNITSELECT;

typedef enum : uint8_t{
    TEMP_UNIT_celcius = 0b0,
    TEMP_UNIT_fahrenheit = 0b1
}BNO_TEMP_UNITSELECT;

typedef enum : uint8_t{
    DATA_UNIT_android = 0b1,
    DATA_UNIT_windows = 0b0
}BNO_DATA_UNITSELECT;

typedef enum : uint8_t{
    // Config
    OPERATIONMODE_CONFIG = 0b0000,

    // Non-Fusion
    OPERATIONMODE_ACCONLY = 0b0001,
    OPERATIONMODE_MAGONLY = 0b0010,
    OPERATIONMODE_GYROONLY = 0b0011,
    OPERATIONMODE_ACCMAG = 0b0100,
    OPERATIONMODE_ACCGYRO = 0b0101,
    OPERATIONMODE_MAGGYRO = 0b0110,
    OPERATIONMODE_AMG = 0b0111,

    // Fusion
    OPERATIONMODE_IMU = 0b1000,
    OPERATIONMODE_COMPASS = 0b1001,
    OPERATIONMODE_M4G = 0b1010,
    OPERATIONMODE_NDOF_OFF = 0b1011,
    OPERATIONMODE_NDOF = 0b1100
}BNO_OPERATIONMODE;

typedef enum : uint8_t{
    BNO_POWER_Normal = 0b00,
    BNO_POWER_LowPower = 0b01,
    BNO_POWER_Suspend = 0b10,
    BNO_POWER_Invalid = 0b11
}BNO_POWERMODE;

typedef enum : uint8_t{
    ACC_SCALE_2g = 0b00,
    ACC_SCALE_4g = 0b01,
    ACC_SCALE_8g = 0b10,
    ACC_SCALE_16g = 0b11
}BNO_ACC_FULLSCALE;

typedef enum : uint8_t{
    ACC_BANDWIDTH_7hz81 = 0b000,
    ACC_BANDWIDTH_15hz63 = 0b001,
    ACC_BANDWIDTH_31hz25 = 0b010,
    ACC_BANDWIDTH_62hz5 = 0b011,
    ACC_BANDWIDTH_125hz = 0b100,
    ACC_BANDWIDTH_250hz = 0b101,
    ACC_BANDWIDTH_500hz = 0b110,
    ACC_BANDWIDTH_1000hz = 0b111
}BNO_ACC_BANDWIDTH;

typedef enum : uint8_t{
    ACC_OPERATION_Normal = 0b000,
    ACC_OPERATION_Suspend = 0b001,
    ACC_OPERATION_LowPower1 = 0b010,
    ACC_OPERATION_Standby = 0b011,
    ACC_OPERATION_LowPower2 = 0b100,
    ACC_OPERATION_DeepSuspend = 0b101
}BNO_ACC_OPERATION;

typedef enum : uint8_t{
    GYRO_SCALE_2000 = 0b000,
    GYRO_SCALE_1000 = 0b001,
    GYRO_SCALE_500 = 0b010,
    GYRO_SCALE_250 = 0b011,
    GYRO_SCALE_125 = 0b100
}BNO_GYRO_FULLSCALE;

typedef enum : uint8_t{
    GYRO_BANDWIDTH_523 = 0b000,
    GYRO_BANDWIDTH_230 = 0b001,
    GYRO_BANDWIDTH_116 = 0b010,
    GYRO_BANDWIDTH_47 = 0b011,
    GYRO_BANDWIDTH_23 = 0b100,
    GYRO_BANDWIDTH_12 = 0b101,
    GYRO_BANDWIDTH_64 = 0b110,
    GYRO_BANDWIDTH_32 = 0b111
}BNO_GYRO_BANDWIDTH;

typedef enum : uint8_t{
    GYRO_OPERATION_Normal = 0b000,
    GYRO_OPERATION_FastPowerUp = 0b001,
    GYRO_OPERATION_DeepSuspend = 0b010,
    GYRO_OPERATION_Suspend = 0b011,
    GYRO_OPERATION_PowerSave = 0b100
}BNO_GYRO_OPERATION;

typedef enum : uint8_t{
    MAG_OUTPUTRATE_2Hz = 0b000,
    MAG_OUTPUTRATE_6Hz = 0b001,
    MAG_OUTPUTRATE_8Hz = 0b010,
    MAG_OUTPUTRATE_10Hz = 0b011,
    MAG_OUTPUTRATE_15Hz = 0b100,
    MAG_OUTPUTRATE_20Hz = 0b101,
    MAG_OUTPUTRATE_25Hz = 0b110,
    MAG_OUTPUTRATE_30Hz = 0b111
}BNO_MAG_OUTPUTRATE;

typedef enum : uint8_t{
    MAG_OPERATION_LowPower = 0b00,
    MAG_OPERATION_Regular = 0b01,
    MAG_OPERATION_EnhancedRegular = 0b10,
    MAG_OPERATION_HighAccuracy = 0b11
}BNO_MAG_OPERATION;

typedef enum : uint8_t{
    MAG_POWER_Normal = 0b00,
    MAG_POWER_Sleep = 0b01,
    MAG_POWER_Suspend = 0b10,
    MAG_POWER_Force = 0b11
}BNO_MAG_POWER;

// Page 0

struct BNO_STR_UNITSELECT{ // Page 0 - 0x3B
    BNO_ACC_UNITSELECT accUnit = ACC_UNIT_m2s; // 0. Bit
    BNO_GYRO_UNITSELECT gyroUnit = GYRO_UNIT_dps; // 1. Bit
    BNO_EULER_UNITSELECT eulerUnit = EULER_UNIT_degree; // 2. Bit
    BNO_TEMP_UNITSELECT tempUnit = TEMP_UNIT_celcius; // 4. Bit
    BNO_DATA_UNITSELECT dataUnit = DATA_UNIT_windows; // 7. Bit
};

struct BNO_STR_OPERATIONMODE{ // Page 0 - 0x3D
    BNO_OPERATIONMODE operationMode = OPERATIONMODE_AMG; // 3-0 Bit
};

struct BNO_STR_POWER_MODE{ // Page 0 - 0x3E
    BNO_POWERMODE powerMode = BNO_POWER_Normal; // 1-0 Bit
};

// Page 1

struct BNO_STR_ACCCONFIG{
    BNO_ACC_FULLSCALE accScale = ACC_SCALE_16g; // 1-0 Bit
    BNO_ACC_BANDWIDTH accBandwidth = ACC_BANDWIDTH_31hz25; // 4-2 Bit
    BNO_ACC_OPERATION accOperation = ACC_OPERATION_Normal; // 7-5 Bit
};

struct BNO_STR_GYRO_CONFIG_0{
    BNO_GYRO_BANDWIDTH gyroBandwidth = GYRO_BANDWIDTH_23; // 5-3 Bit
    BNO_GYRO_FULLSCALE gyroScale = GYRO_SCALE_2000; // 2-0 Bit
};

struct BNO_STR_GYRO_CONFIG_1{
    BNO_GYRO_OPERATION gyroOperation = GYRO_OPERATION_Normal; // 2-0 Bit
};

struct BNO_STR_MAG_CONFIG{
    BNO_MAG_OUTPUTRATE magOutputRate = MAG_OUTPUTRATE_30Hz; // 2-0 Bit
    BNO_MAG_OPERATION magOperation = MAG_OPERATION_Regular; // 4-3 Bit
    BNO_MAG_POWER magPower = MAG_POWER_Normal; // 6-5 Bit
};

struct BNO_STR_INT_EN{
    BNO_GENERALONOFF accNoMotion = BNO_OFF; // 7. Bit
    BNO_GENERALONOFF accAnyMotion = BNO_OFF; // 6. Bit
    BNO_GENERALONOFF accHighG = BNO_OFF; // 5. Bit
    BNO_GENERALONOFF gyroDataReady = BNO_OFF; // 4. Bit
    BNO_GENERALONOFF gyroHighRate = BNO_OFF; // 3. Bit
    BNO_GENERALONOFF gyroAnyMotion = BNO_OFF; // 2. Bit
    BNO_GENERALONOFF magDataReady = BNO_OFF; // 1. Bit
    BNO_GENERALONOFF accDataReady = BNO_OFF; // 0. Bit
};

struct BNO_STR_ACC_INT_SETTINGS{
    BNO_GENERALONOFF HighG_X = BNO_OFF; // 7. Bit
    BNO_GENERALONOFF HighG_Y = BNO_OFF; // 6. Bit
    BNO_GENERALONOFF HighG_Z = BNO_OFF; // 5. Bit
    BNO_GENERALONOFF AnyMotion_X = BNO_OFF; // 4. Bit
    BNO_GENERALONOFF AnyMotion_Y = BNO_OFF; // 3. Bit
    BNO_GENERALONOFF AnyMotion_Z = BNO_OFF; // 2. Bit
    uint8_t AnyMotionDuration = 0; // 1-0 Bit
};

struct BNO_STR_PAGE_0{
    BNO_STR_UNITSELECT unitSelection;
    BNO_STR_OPERATIONMODE operationMode;
    BNO_STR_POWER_MODE powerMode;
};

struct BNO_STR_PAGE_1{
    BNO_STR_ACCCONFIG accConfig;
    BNO_STR_GYRO_CONFIG_0 gyroConfig;
    BNO_STR_GYRO_CONFIG_1 gyroConfig_1;
    BNO_STR_MAG_CONFIG magConfig;
    BNO_STR_INT_EN intEnable;
    BNO_STR_ACC_INT_SETTINGS accIntSettings;
};

struct BNO_STR_REGISTERS{
    BNO_STR_PAGE_0 registersPage_0;
    BNO_STR_PAGE_1 registersPage_1;
};

struct BNO_DOF3_Float{
    float x;
    float y;
    float z;
};

struct BNO_DOF3_int16{
    int16_t x;
    int16_t y;
    int16_t z;
};

union int16toint8{
    int16_t int16var;
    uint8_t int8var[2];
};

class BNO055{
private:
    BNO_STR_REGISTERS _configs;

    typedef struct _params{

    }_params;

    BNO_OPERATIONMODE getOperationMode(void);
    void changePage(uint8_t page);
    void getCalibrationStatus(uint8_t *systemCalib, uint8_t *gyroCalib, uint8_t *accCalib, uint8_t *magCalib);
    void getRawGyroData(BNO_DOF3_int16 *gyroData);
    void BNO_GyroCalibration();
    void setOperationMode(void) const;
    void setRegUnits(void) const;
    void setRegPowerMode(void) const;
    void setRegAccConfig(void) const;
    void setRegGyroConfig(void) const;
    void setRegMagConfig(void) const;
    void setRegIntEnable(void) const;
    void setRegAccIntSettings();
    bool BNO_I2CWriteByte(uint8_t regadr, uint8_t data);
    uint8_t BNO_I2CReadByte(uint8_t regadr, uint8_t *temp);
    uint8_t BNO_I2CReadBytes(uint8_t regadr, uint8_t *temp, uint8_t length);

public:
    void begin(BNO_STR_REGISTERS registersSet);
    BNO_DOF3_Float getGyroData();
    BNO_DOF3_Float getMagData();
    BNO_DOF3_Float getAccData();
    void BnoSetExternalClock();
    void BnoReset();
};

#endif