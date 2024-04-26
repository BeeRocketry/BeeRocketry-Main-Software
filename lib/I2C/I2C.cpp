#include <Arduino.h>

#define STM_I2C2_FREQ 400000
#define DATA_TO_SEND_LENGHT 30

#include <Wire.h>

#define I2C2_SDA PB11
#define I2C2_SCL PB12

void I2Cinit(void){
    Wire.setSDA(I2C2_SDA);
    Wire.setSCL(I2C2_SCL);
    Wire.begin();
}

void I2CWriteReg(int8_t chipadr, int8_t regadr, int8_t data){
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.write(data);
    Wire.endTransmission();
}

void I2CReadReg(int8_t chipadr, int8_t regadr, byte *temp){
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.requestFrom(chipadr, 1);

    if(Wire.available() <= 1){
        *temp = Wire.read();
    }
}