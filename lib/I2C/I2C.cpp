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
    Wire.setClock(400000);
}

int8_t I2CWriteReg(int8_t chipadr, int8_t regadr, int8_t data){
    byte result;
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.write(data);
    result = Wire.endTransmission();
    return result;
}

void I2CReadReg(int8_t chipadr, int8_t regadr, int32_t *temp){
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.requestFrom(chipadr, 1);

    if(Wire.available()){
        *temp = Wire.read();
    }
}

void I2CReadRegMulti(int8_t chipadr, int8_t regadr, uint8_t temp[], int length){
    int cnt = 0;
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.requestFrom(chipadr, length);

    while(Wire.available() < length && cnt < length){
        temp[cnt++] = Wire.read();
    }
}