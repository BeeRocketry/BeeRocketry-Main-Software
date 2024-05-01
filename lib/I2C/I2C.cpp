#include <Arduino.h>

#define STM_I2C2_FREQ 400000
#define DATA_TO_SEND_LENGHT 30

#include <Wire.h>

#define I2C2_SDA PB7
#define I2C2_SCL PB6

void I2Cinit(void){
    Wire.setSDA(I2C2_SDA);
    Wire.setSCL(I2C2_SCL);
    Wire.begin();
    Wire.setClock(400000);
}

int8_t I2CWriteByte(int8_t chipadr, int8_t regadr, int8_t data){
    int8_t result;
    
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.write(data);
    result = Wire.endTransmission();
    return result;
}

int8_t I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint16_t timeout){
    int8_t cnt = 0;
    uint8_t length = 1;
    uint32_t t1 = millis();
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.beginTransmission(chipadr);
    Wire.requestFrom(chipadr, 1);

    while(Wire.available() && (timeout == 0 || millis() - t1 < timeout)){
        *temp = Wire.read();
        cnt++;
    }

    Wire.endTransmission();

    if(timeout > 0 && millis() - t1 >= timeout && cnt < length){
        cnt = -1;
    }

    return cnt;
}

int8_t I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t *temp, uint8_t length, uint16_t timeout){
    int8_t cnt = 0;
    uint32_t t1 = millis();
    Wire.beginTransmission(chipadr);
    Wire.write(regadr);
    Wire.endTransmission();

    Wire.beginTransmission(chipadr);
    Wire.requestFrom(chipadr, length);

    while(Wire.available() && (timeout == 0 || millis() - t1 < timeout)){
        temp[cnt++] = Wire.read();
    }

    Wire.endTransmission();

    if(timeout > 0 && millis() - t1 >= timeout && cnt < length){
        cnt = -1;
    }

    return cnt;
}