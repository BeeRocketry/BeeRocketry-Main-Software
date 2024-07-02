#include "MPU.h"
#include "MPURegister.h"

float Mag_Resolution = 0;

float Mag_bias[3] = {0, 0, 0};
float Mag_bias_factory[3] = {0, 0, 0};
float Mag_scale[3] = {1., 1., 1.};
float magnetic_declination = 6.14; // Ankara

// MAGNETOMETER
// Mag çözünürlüğünü değer olarak döndürür. Yukarıdaki mag çözünürlük ile aynı şeydir.
float getMagRes(MAG_OUTPUTBIT bitmode){
    switch (bitmode)
    {
    case MAG_OUTPUTBIT_14BIT:
        return 10. * 4912. / 8190.0;

    case MAG_OUTPUTBIT_16BIT:
        return 10. * 4912. / 32760.0;
    
    default:
        return 0;
    }
}

// Magnetometer'in FuseROM'da bulunan fabrika bias verilerini alır ve işler.
Status getMagASData(struct MPU_REGISTERS *settings){
    DEBUG_PRINTLN(F("Mag Bias Factory Değer alimi baslatiliyor..."));
    delay(1000);

    uint8_t reg = 0;
    uint8_t rawData[3] = {0, 0, 0};

    settings->MAG_Control_Register.operationMode = MAG_OPERATIONMODE_FUSEROM;

    reg = ((uint8_t)settings->MAG_Control_Register.operationMode);
    I2CWriteByte(MAG_CHIPADR, MAG_CTRL, reg);

    delay(20);

    I2CReadBytes(MAG_CHIPADR, MAG_ASAX, rawData, 3, TIMEOUT_I2C);
    
    Mag_bias_factory[0] = (float)(rawData[0] - 128) * 256. + 1.;
    Mag_bias_factory[1] = (float)(rawData[1] - 128) * 256. + 1.;
    Mag_bias_factory[2] = (float)(rawData[2] - 128) * 256. + 1.;

    delay(20);

    settings->MAG_Control_Register.operationMode = MAG_OPERATIONMODE_CONTINUOUSMEASURE2;
    I2CWriteByte(MAG_CHIPADR, MAG_CTRL, reg);

    delay(20);

    DEBUG_PRINTLN(F("Mag Bias Factory"));
    DEBUG_PRINT(F(" X:"));
    DEBUG_PRINTLN(F(Mag_bias_factory[0]));
    DEBUG_PRINT(F(" Y:"));
    DEBUG_PRINTLN(F(Mag_bias_factory[1]));
    DEBUG_PRINT(F(" Z:"));
    DEBUG_PRINTLN(F(Mag_bias_factory[2]));
    DEBUG_PRINTLN();

    delay(2000);

    return MPU_Success;
}

/*
    Mag verilerini işlemek için gerekli olan Mag_bias ve Mag_scale verilerini
    bulur. Bunun için belirli bir süre çalışacak bir fonksiyon çalıştırır.
    Maksimum ve minimum değerleri bularak bunlardan bias ve scale verilerini
    çıkarır.
*/
void collectMagDataTo(void){
    DEBUG_PRINTLN(F("Magnetometer Kalibrasyonu için Data toplama baslatiliyor..."));
    DEBUG_PRINTLN(F("Cihazi 8 seklinde döndürünüz..."));

    delay(1000);

    DEBUG_PRINTLN(F("Baslatildi..."));

    uint16_t samplecount = 3000;

    int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767, 32767};
    int16_t mag_temp[3] = {0, 0, 0};
    Dof3Data_Int data;

    for(uint16_t i = 0; i < samplecount; i++){
        if(samplecount % 50 == 0){
            DEBUG_PRINT(F("Örnek "));
            DEBUG_PRINTLN(F(samplecount));
        }
        getMagData(&data);

        mag_temp[0] = data.x;
        mag_temp[1] = data.y;
        mag_temp[2] = data.z;

        for(int j = 0; j < 3; j++){
            if(mag_temp[j] > mag_max[j])
                mag_max[j] = mag_temp[j];
            if(mag_temp[j] < mag_min[j])
                mag_min[j] = mag_temp[j];
        }
        delay(12);
    }

    DEBUG_PRINTLN(F("Örnekleme Tamamlandi..."));
    delay(1000);

    bias[0] = (mag_max[0] + mag_min[0]) / 2;
    bias[1] = (mag_max[1] + mag_min[1]) / 2;
    bias[2] = (mag_max[2] + mag_min[2]) / 2;

    float bias_res = getMagRes(MAG_OUTPUTBIT_16BIT);

    Mag_bias[0] = (float)bias[0] * bias_res * Mag_bias_factory[0];
    Mag_bias[1] = (float)bias[1] * bias_res * Mag_bias_factory[1];
    Mag_bias[2] = (float)bias[2] * bias_res * Mag_bias_factory[2];

    scale[0] = (float)(mag_max[0] - mag_min[0]) * Mag_bias_factory[0] / 2;
    scale[1] = (float)(mag_max[1] - mag_min[1]) * Mag_bias_factory[1] / 2;
    scale[2] = (float)(mag_max[2] - mag_min[2]) * Mag_bias_factory[2] / 2;

    float avg_rad = scale[0] + scale[1] + scale[2];
    avg_rad /= 3;

    Mag_scale[0] = avg_rad / ((float)scale[0]);
    Mag_scale[1] = avg_rad / ((float)scale[1]);
    Mag_scale[2] = avg_rad / ((float)scale[2]);

    DEBUG_PRINTLN(F("Mag Bias"));
    DEBUG_PRINT(F(" X:"));
    DEBUG_PRINTLN(F(Mag_bias[0]));
    DEBUG_PRINT(F(" Y:"));
    DEBUG_PRINTLN(F(Mag_bias[1]));
    DEBUG_PRINT(F(" Z:"));
    DEBUG_PRINTLN(F(Mag_bias[2]));
    DEBUG_PRINTLN();
    DEBUG_PRINTLN(F("Mag Scale"));
    DEBUG_PRINT(F(" X:"));
    DEBUG_PRINTLN(F(Mag_scale[0]));
    DEBUG_PRINT(F(" Y:"));
    DEBUG_PRINTLN(F(Mag_scale[1]));
    DEBUG_PRINT(F(" Z:"));
    DEBUG_PRINTLN(F(Mag_scale[2]));
    DEBUG_PRINTLN();

    delay(2000);
}

// Mag çözünürlüğünü verir.
Status setMagScalingFactor(struct MPU_REGISTERS *settings){
    DEBUG_PRINTLN(F("Mag Scaling Faktör Ayarlaniyor..."));
    switch (settings->MAG_Control_Register.outputBit)
    {
    case 0b0:
        Mag_Resolution = 10. * 4912. / 8190.0;
        break;
    
    case 0b1:
        Mag_Resolution = 10. * 4912. / 32760.0;
        break;

    default:
        DEBUG_PRINTLN(F("Mag Scaling Faktör girdisi yanlis..."));
        return MPU_Error;
    }
    DEBUG_PRINTLN(F("Mag Scaling Faktör olarak ayarlandi..."));
    return MPU_Success;
}