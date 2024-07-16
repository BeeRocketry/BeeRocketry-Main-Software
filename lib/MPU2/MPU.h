#include <Arduino.h>
#include "I2C.h"
#include "debugprinter.h"

#define SDA_PIN PA1
#define SCL_PIN PB2

#define MPU_CHIPADR 0x68
#define MMC5603_CHIPADR 0x30

#define WHOAMI 0x75

#define CONFIG 0x1A
#define CONFIG_GYRO 0x1B
#define CONFIG_ACCEL 0x1C
#define CONFIG_ACCEL_2 0x1D
#define POWER_MANAGEMENT 0x6B
#define POWER_MANAGEMENT_2 0x6C
#define USER_CTRL 0x6A
#define MAG_STATUS 0x02
#define MAG_STATUS_2 0x09
#define MAG_CTRL 0x0A
#define SAMPLERATE_DIVIDER 0x19
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define I2C_MASTER_CONTROL 0x24
#define FIFOENABLE_REG 0x23
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74

#define GYRO_X_OFFSET_MSB 0x13
#define GYRO_X_OFFSET_LSB 0x14
#define GYRO_Y_OFFSET_MSB 0x15
#define GYRO_Y_OFFSET_LSB 0x16
#define GYRO_Z_OFFSET_MSB 0x17
#define GYRO_Z_OFFSET_LSB 0x18

#define ACCEL_X_OFFSET_MSB 0x77
#define ACCEL_X_OFFSET_LSB 0x78
#define ACCEL_Y_OFFSET_MSB 0x7A
#define ACCEL_Y_OFFSET_LSB 0x7B
#define ACCEL_Z_OFFSET_MSB 0x7D
#define ACCEL_Z_OFFSET_LSB 0x7E

#define ACCEL_X_OUTPUT_MSB 0x3B
#define ACCEL_X_OUTPUT_LSB 0x3C
#define ACCEL_Y_OUTPUT_MSB 0x3D
#define ACCEL_Y_OUTPUT_LSB 0x3E
#define ACCEL_Z_OUTPUT_MSB 0x3F
#define ACCEL_Z_OUTPUT_LSB 0x40

#define TEMP_OUTPUT_MSB 0x41
#define TEMP_OUTPUT_LSB 0x42

#define GYRO_X_OUTPUT_MSB 0x43
#define GYRO_X_OUTPUT_LSB 0x44
#define GYRO_Y_OUTPUT_MSB 0x45
#define GYRO_Y_OUTPUT_LSB 0x46
#define GYRO_Z_OUTPUT_MSB 0x47
#define GYRO_Z_OUTPUT_LSB 0x48

#define MMC_X_OUTPUT_MSB 0x00
#define MMC_X_OUTPUT_LSB 0x01
#define MMC_Y_OUTPUT_MSB 0x02
#define MMC_Y_OUTPUT_LSB 0x03
#define MMC_Z_OUTPUT_MSB 0x04
#define MMC_Z_OUTPUT_LSB 0x05

#define MMC_X_OUTPUT_Extra 0x06
#define MMC_Y_OUTPUT_Extra 0x07
#define MMC_Z_OUTPUT_Extra 0x08

#define MMC_Temp_OUTPUT 0x09

#define MMC_Status 0x18
#define MMC_ODR 0x1A
#define MMC_Control0 0x1B
#define MMC_Control1 0x1C
#define MMC_Control2 0x1D
#define MMC_ProductID 0x39

extern float Acc_Resolution;
extern float AccRange;
extern float Gyro_Resolution;
extern float GyroRange;
extern float Mag_Resolution;

extern float Acc_bias[3];
extern float Gyro_bias[3];
extern float Mag_bias[3];
extern float Mag_bias_factory[3];
extern float Mag_scale[3];
extern float magnetic_declination;
extern const uint16_t CALIB_ACCEL_SENSIVITY;

typedef enum Status{
    MPU_Success = 0,
    MPU_Timeout,
    MPU_Error
}Status;

typedef enum MPU_FIFOMode{
    FIFO_Overwrite = 0b0,
    FIFO_Hold = 0b1
}MPU_FIFOMode;

typedef enum MPU_SYNC{
    SYNC_noSync = 0b000,
    SYNC_FSync = 0b001,
    SYNC_TempFIFO = 0b010,
    SYNC_GyroX = 0b011,
    SYNC_GyroY = 0b100,
    SYNC_GyroZ = 0b101,
    SYNC_AccX = 0b110,
    SYNC_AccY = 0b111
}MPU_SYNC;

typedef enum MPU_DLPF{
    GYRO_DLPF_0 = 0b000,
    GYRO_DLPF_1 = 0b001,
    GYRO_DLPF_2 = 0b010,
    GYRO_DLPF_3 = 0b011,
    GYRO_DLPF_4 = 0b100,
    GYRO_DLPF_5 = 0b101,
    GYRO_DLPF_6 = 0b110,
}MPU_DLPF;

typedef enum MPU_GYRO_Scale{
    GYROScale_250 = 0b00,
    GYROScale_500 = 0b01,
    GYROScale_1000 = 0b10,
    GYROScale_2000 = 0b11,
}MPU_GYRO_Scale;

typedef enum MPU_FChoice{
    GYRO_FChoice_DLPFOn = 0b00,
    GYRO_FChoice_DLPFGyroOn = 0b10,
    GYRO_FChoice_DLPFOff = 0b01
}MPU_FChoice;

typedef enum MPU_ACC_Scale{
    ACCScale_2G = 0b00,
    ACCScale_4G = 0b01,
    ACCScale_8G = 0b10,
    ACCScale_16G = 0b11,
}MPU_ACC_Scale;

typedef enum MPU_ACC_Fchoice{
    ACC_FChoice_DLPFOn = 0b0,
    ACC_FChoice_DLPFOff = 0b1,
}MPU_ACC_Fchoice;

typedef enum MPU_ACC_DLPF{
    ACC_DLPF_0 = 0b000,
    ACC_DLPF_1 = 0b001,
    ACC_DLPF_2 = 0b010,
    ACC_DLPF_3 = 0b011,
    ACC_DLPF_4 = 0b100,
    ACC_DLPF_5 = 0b101,
    ACC_DLPF_6 = 0b110,
}MPU_ACC_DLPF;

typedef enum I2C_MASTER_CLOCK{
    I2C_CLOCK_348 = 0b0000,
    I2C_CLOCK_333 = 0b0001,
    I2C_CLOCK_320 = 0b0010,
    I2C_CLOCK_308 = 0b0011,
    I2C_CLOCK_296 = 0b0100,
    I2C_CLOCK_286 = 0b0101,
    I2C_CLOCK_276 = 0b0110,
    I2C_CLOCK_267 = 0b0111,
    I2C_CLOCK_258 = 0b1000,
    I2C_CLOCK_500 = 0b1001,
    I2C_CLOCK_471 = 0b1010,
    I2C_CLOCK_444 = 0b1011,
    I2C_CLOCK_421 = 0b1100,
    I2C_CLOCK_400 = 0b1101,
    I2C_CLOCK_381 = 0b1110,
    I2C_CLOCK_364 = 0b1111
}I2C_MASTER_CLOCK;

typedef enum MPU_BYPASS{
    MPU_BypassOn = 0b1,
    MPU_BypassOff = 0b0
}MPU_BYPASS;

typedef enum MPU_Sleep{
    MPU_sleep = 0b0,
    MPU_active = 0b1
}MPU_Sleep;

typedef enum MPU_CLOCK_SELECT{
    MPU_CLOCK_8MHzInternal = 0b000,
    MPU_CLOCK_GYRO_X = 0b001,
    MPU_CLOCK_GYRO_Y = 0b010,
    MPU_CLOCK_GYRO_Z = 0b011,
    MPU_CLOCK_32768 = 0b101,
    MPU_CLOCK_192 = 0b110,
}MPU_CLOCK_SELECT;

typedef enum MPU_GENERAL_ONOFF{
    MPU_General_ON = 0b1,
    MPU_General_OFF = 0b0
}MPU_GENERAL_ONOFF;

struct MPU_CONFIG_REG{
    MPU_FIFOMode fifoMode = FIFO_Overwrite; // 6. bit
    MPU_SYNC syncMode = SYNC_noSync; // 5-3 bit
    MPU_DLPF dlpfMode = GYRO_DLPF_3; // 2-0 bit
};

struct MPU_GYRO_CONFIG_REG{
    MPU_GYRO_Scale gyroScale = GYROScale_1000; // 4-3 bit
    MPU_FChoice dlpfchoice = GYRO_FChoice_DLPFOn; // 1-0 bit
};

struct MPU_ACC_CONFIG1_REG{
    MPU_ACC_Scale accScale = ACCScale_16G; // 4-3 bit
};

struct MPU_ACC_CONFIG2_REG{
    MPU_ACC_Fchoice accDLPFmode = ACC_FChoice_DLPFOn; // 3. bit
    MPU_ACC_DLPF accDLPFSettings = ACC_DLPF_1; // 2-0 bit
};

struct MPU_I2C_MASTER_REG{
    I2C_MASTER_CLOCK masterClock = I2C_CLOCK_400;
};

struct MPU_INT_PIN_CFG_REG{
    MPU_BYPASS i2cbypassMode = MPU_BypassOn; // 1. bit
};

struct MPU_USER_CONTROL_REG{
    MPU_GENERAL_ONOFF fifoEnable = MPU_General_OFF; // 6. bit
    MPU_GENERAL_ONOFF i2cMasterEnable = MPU_General_OFF; // 5. bit
};

struct MPU_POWER_MANAGAMENT1_REG{
    MPU_Sleep activationmode = MPU_active;
    MPU_CLOCK_SELECT clockSet = MPU_CLOCK_GYRO_X;
};

struct MPU_REGISTERS{
    MPU_CONFIG_REG MPU_Config_Register;
    MPU_GYRO_CONFIG_REG MPU_GyroConfig_Register;
    MPU_ACC_CONFIG1_REG MPU_AccConfig_Register;
    MPU_ACC_CONFIG2_REG MPU_AccConfig_2_Register;
    MPU_I2C_MASTER_REG MPU_I2CMaster_Register;
    MPU_INT_PIN_CFG_REG MPU_INTPin_Register;
    MPU_USER_CONTROL_REG MPU_UserControl_Register;
    MPU_POWER_MANAGAMENT1_REG MPU_PowerManagament1_Register;
};

typedef union Float2Byte{
    float DataFloat;
    uint8_t DataByte[4];
}Float2Byte;

typedef union Int2Byte{
    int16_t DataInt;
    uint8_t DataByte[4];
}Int2Byte;

typedef struct Dof3Data_Float{
    float x = 0;
    float y = 0;
    float z = 0;
}Dof3Data_Float;

typedef struct Dof3Data_Int{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
}Dof3Data_Int;

typedef struct Dof3Data_IntMAG{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
}Dof3Data_IntMAG;


// Prototypes

// MagCalibration.cpp Prototypes
Status getMagASData(struct MPU_REGISTERS *settings);
void collectMagDataTo(void);
Status setMagScalingFactor(struct MPU_REGISTERS *settings);

// MPUCalibration.cpp Prototypes
Status setGyroScalingFactor(struct MPU_REGISTERS *settings);
Status setAccelScalingFactor(struct MPU_REGISTERS *settings);
void CalibrationAccGyro(struct MPU_REGISTERS *settings);
void AccGyroCalibRegister();
void AccGyroDataCollection();
void writeAccOffsets();
void writeGyroOffsets();

// MPU.cpp Prototypes
Status setScalingFactors(struct MPU_REGISTERS *settings);
Status getRawAccGyroTempData(Dof3Data_Int *rawDataAccel, Dof3Data_Int *rawDataGyro, int16_t *temp);
Status normalizeAccGyroTempData(Dof3Data_Int *rawDataAccel, Dof3Data_Int *rawDataGyro, int16_t *tempint, float *temp, Dof3Data_Float *accel, Dof3Data_Float *gyro);
Status getMagData(Dof3Data_Int *data);
Status normalizeMagData(Dof3Data_Int *data, Dof3Data_Float *mag);
Status mpuInit(struct MPU_REGISTERS *settings);