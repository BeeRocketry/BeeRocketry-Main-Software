#include <I2Class.h>

uint8_t I2Class::I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t* temp, uint16_t timeout){
    return this->I2CReadBytes(chipadr, regadr, temp, 1, timeout);
}

uint8_t I2Class::I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout){
    if(!this->isSet){
        DEBUG_PRINTLN(F("I2C ayarlamasi yapilmadi..."));
        return -1;
    }

    uint8_t cnt = 0;
    uint32_t t1 = millis();

    for(uint8_t check = 0; check < length; check += minimum(length, BUFFER_LENGTH)){
        this->wireI2C->beginTransmission(chipadr);
        this->wireI2C->write(regadr + check);
        this->wireI2C->endTransmission(false);

        this->wireI2C->requestFrom(chipadr, minimum(length - check, BUFFER_LENGTH));

        uint32_t start = millis();
        while(this->wireI2C->available()){
            if(timeout == 0 || millis() - start > timeout){
                DEBUG_PRINTLN(F("I2C ReadByte timeout süresini asti..."));
                return -1;
            }

            temp[cnt++] = this->wireI2C->read();
        }
    }

    return cnt;
}

bool I2Class::I2CWriteByte(uint8_t chipadr, uint8_t regadr, uint8_t data){
    if(!this->isSet){
        DEBUG_PRINTLN(F("I2C ayarlamasi yapilmadi..."));
        return false;
    }   
    
    int8_t result;

    this->wireI2C->beginTransmission(chipadr);
    this->wireI2C->write(regadr);
    this->wireI2C->write(data);
    result = this->wireI2C->endTransmission(true);
    return result == 0;
}

uint8_t I2Class::minimum(uint8_t x, uint8_t y){
    return (x < y) ? x : y;
}

I2Class::I2Class() : created(false){

}

I2Class::I2Class(uint8_t sda, uint8_t scl) : isSet(true), created(true){
    this->wireI2C = new TwoWire();

    this->wireI2C->begin(sda, scl);
    this->wireI2C.setClock(400000);
}

I2Class::~I2Class(){
    if(created && this->wireI2C != nullptr){
        delete this->wireI2C;
    }
}