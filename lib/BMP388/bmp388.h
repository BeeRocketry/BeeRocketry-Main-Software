#include <Arduino.h>
#include "I2C.h"

#define SeaLevelhPa 1013.25

// Durum ve Ayar Registerları
#define REG_FIFO_CONFIG1 0x17
#define REG_FIFO_CONFIG2 0x18
#define REG_PWR_CTRL 0x1B
#define REG_OSR 0x1C
#define CHIP_ADR 0x76
#define REG_WhoAmI 0x00
#define REG_STATUS 0x03
#define REG_ODR 0x1D
#define REG_CONFIG 0x1F
#define REG_CMD 0x7E

// Basınç Registerları
#define REG_PRESS_XLSB 0x04
#define REG_PRESS_LSB 0x05
#define REG_PRESS_MSB 0x06

// Sıcaklık Registerları
#define REG_TEMP_XLSB 0x07
#define REG_TEMP_LSB 0x08
#define REG_TEMP_MSB 0x09

// Sıcaklık Katsayıları
// T1 -- 16 Bit Unsigned
#define REG_TEMP_T1_LSB 0x31
#define REG_TEMP_T1_MSB 0x32
// T2 -- 16 Bit Unsigned
#define REG_TEMP_T2_LSB 0x33
#define REG_TEMP_T2_MSB 0x34
// T3 -- 8 Bit Signed
#define REG_TEMP_T3 0x35

// Basınç Katsayıları
// P1 -- 16 Bit Signed
#define REG_PRESS_P1_LSB 0x36
#define REG_PRESS_P1_MSB 0x37
// P2 -- 16 Bit Signed
#define REG_PRESS_P2_LSB 0x38
#define REG_PRESS_P2_MSB 0x39
// P3 -- 8 Bit Signed
#define REG_PRESS_P3 0x3A
// P4 -- 8 Bit Signed
#define REG_PRESS_P4 0x3B
// P5 -- 16 Bit Unsigned
#define REG_PRESS_P5_LSB 0x3C
#define REG_PRESS_P5_MSB 0x3D
// P6 -- 16 Bit Unsigned
#define REG_PRESS_P6_LSB 0x3E
#define REG_PRESS_P6_MSB 0x3F
// P7 -- 8 Bit Signed
#define REG_PRESS_P7 0x40
// P8 -- 8 Bit Signed
#define REG_PRESS_P8 0x41
// P9 -- 16 Bit Signed
#define REG_PRESS_P9_LSB 0x42
#define REG_PRESS_P9_MSB 0x43
// P10 -- 8 Bit Signed
#define REG_PRESS_P10 0x44
// P11 -- 8 Bit Signed
#define REG_PRESS_P11 0x45


typedef enum BMP_ONOFF{
    BMP_ON = 0b1,
    BMP_OFF = 0b0
}BMP_ONOFF;

typedef enum BMP_Mode{
    BMP_NormalMode = 0b11,
    BMP_ForcedMode = 0b01,
    BMP_Sleep = 0b00
}BMP_Mode;

typedef enum BMP_Oversampling{
    BMP_OverSampling_1x = 0b000,
    BMP_OverSampling_2x = 0b001,
    BMP_OverSampling_4x = 0b010,
    BMP_OverSampling_8x = 0b011,
    BMP_OverSampling_16x = 0b100,
    BMP_OverSampling_32x = 0b101,
}BMP_Oversampling;

typedef enum BMP_IIR_Sampling{
    BMP_IIR_OFF = 0b000,
    BMP_IIR_1X = 0b001,
    BMP_IIR_3X = 0b010,
    BMP_IIR_7X = 0b011,
    BMP_IIR_15X = 0b100,
    BMP_IIR_31X = 0b101,
    BMP_IIR_63X = 0b110,
    BMP_IIR_127X = 0b111
}BMP_IIR_Sampling;

typedef enum BMP_ODR{
    BMP_ODR_5ms = 0b000,
    BMP_ODR_10ms = 0b001,
    BMP_ODR_20ms = 0b010,
    BMP_ODR_40ms = 0b011,
    BMP_ODR_80ms = 0b100,
    BMP_ODR_160ms = 0b101,
    BMP_ODR_320ms = 0b110,
    BMP_ODR_640ms = 0b111
}BMP_ODR;

typedef struct{
    double P1;
    double P2;
    double P3;
    double P4;
    double P5;
    double P6;
    double P7;
    double P8;
    double P9;
    double P10;
    double P11;
    double T1;
    double T2;
    double T3;
    float tfine;
}BMP_QuantizedCalibData;

typedef struct{
    int16_t P1;
    int16_t P2;
    int8_t P3;
    int8_t P4;
    uint16_t P5;
    uint16_t P6;
    int8_t P7;
    int8_t P8;
    int16_t P9;
    int8_t P10;
    int8_t P11;
    uint16_t T1;
    uint16_t T2;
    int8_t T3;
}BMP_CalibData;

extern BMP_CalibData *CalibDataPointer;
extern BMP_QuantizedCalibData *QuantizedDataPointer;

void BMPInit();
void getCalibrationData();
void setFIFOConfig1(BMP_ONOFF fifoMode);
void setPowerControl(BMP_ONOFF pressureEnable, BMP_ONOFF tempEnable, BMP_Mode bmpMode);
void setOSR(BMP_Oversampling pressureOversampling, BMP_Oversampling tempOversampling);
void setControl(BMP_IIR_Sampling iirSampling);
void setODR(BMP_ODR odrRate);
void setReset();
void getRawData(uint32_t *rawpress, uint32_t *rawtemperature);
float compensatedTempData(uint32_t rawTemperature);
float compensatedPressData(uint32_t rawPressure);
float convertPress2Altitude(float pressure);
