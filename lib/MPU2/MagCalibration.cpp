#include "MPU.h"
#include "MPURegister.h"

float Mag_Resolution = 0;

float Mag_bias[3] = {0, 0, 0};
float Mag_bias_factory[3] = {0, 0, 0};
float Mag_scale[3] = {1., 1., 1.};
float magnetic_declination = 6.14; // Ankara

// MAGNETOMETE
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