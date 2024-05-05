#include "MPU9250.h"

void mpuInit(void){
    setClock(0b1);
    setGyroRange(0b01);
    setAccelRange(0b11);
    setSleep(false);
    Serial2.println("MPU Port Aktifleştirildi.");
    Serial2.print("MPU I2C Adress: 0x");
    Serial2.println(getDeviceID(), HEX);
}

bool testConnect(){
    return getDeviceID() == 0x68;
}

uint8_t getDeviceID(){
    uint8_t temp;
    I2CReadByte(MPU_CHIPADR, WHOAMI, &temp, TIMEOUT_I2C);

    return temp;
}

// SampleRate Register
void setRate(uint8_t rate){
    I2CWriteByte(MPU_CHIPADR, SAMPLERATE_DIVIDER, rate);
}

// Config Register
void setConfig(uint8_t binary){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, CONFIG, &b, TIMEOUT_I2C);
    b = b | (binary << 3);
    I2CWriteByte(MPU_CHIPADR, CONFIG, b);
}

// Gyro Range
void setGyroRange(uint8_t range){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, CONFIG_GYRO, &b, TIMEOUT_I2C);
    b = b | (range << 3);
    I2CWriteByte(MPU_CHIPADR, CONFIG_GYRO, b);
}

// Accel Range
void setAccelRange(uint8_t range){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, CONFIG_ACCEL, &b, TIMEOUT_I2C);
    b = b | (range << 3);
    I2CWriteByte(MPU_CHIPADR, CONFIG_ACCEL, b);
}

// Accel DLPH
void setAccelDLPH(uint8_t binary){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, CONFIG_ACCEL_2, &b, TIMEOUT_I2C);
    b = b | binary;
    I2CWriteByte(MPU_CHIPADR, CONFIG_ACCEL_2, b);
}

void setSleep(bool cond){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, POWER_MANAGEMENT, &b, TIMEOUT_I2C);
    b = b & (cond << 6);
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, b);
}

void setClock(uint8_t bits){
    uint8_t b;
    I2CReadByte(MPU_CHIPADR, POWER_MANAGEMENT, &b, TIMEOUT_I2C);
    b = b | bits;
    I2CWriteByte(MPU_CHIPADR, POWER_MANAGEMENT, b);
}

void getRawAccel(int16_t *accel){
    int16_t accelx, accely, accelz;
    uint8_t buff[6];
    I2CReadBytes(MPU_CHIPADR, ACCEL_X_OUTPUT_MSB, buff, 6, TIMEOUT_I2C);

    accelx = (((int16_t)buff[0]) << 8) | buff[1];
    accely = (((int16_t)buff[2]) << 8) | buff[3];
    accelz = (((int16_t)buff[4]) << 8) | buff[5];

    accel[0] = accelx;
    accel[1] = accely;
    accel[2] = accelz;
}

void getRawGyro(int16_t *gyro){
    int16_t gyrox, gyroy, gyroz;
    uint8_t buff[6];

    I2CReadBytes(MPU_CHIPADR, GYRO_X_OUTPUT_MSB, buff, 6, TIMEOUT_I2C);

    gyrox = (((int16_t)buff[0]) << 8) | buff[1];
    gyroy = (((int16_t)buff[2]) << 8) | buff[3];
    gyroz = (((int16_t)buff[4]) << 8) | buff[5];

    gyro[0] = gyrox;
    gyro[1] = gyroy;
    gyro[2] = gyroz;
}

void getRawMag(int16_t *mag){
    uint8_t buff[6];
    int16_t magx, magy, magz;

    I2CWriteByte(MPU_CHIPADR, INT_PIN_CFG, 0x02);
    delay(10);
    I2CWriteByte(MAG_CHIPADR, 0x0A, 0x01);
    delay(10);
    I2CReadBytes(MAG_CHIPADR, MAG_X_OUTPUT_LSB, buff, 6, TIMEOUT_I2C);
    
    magx = (((int16_t)buff[1]) << 8) | buff[0];
    magy = (((int16_t)buff[3]) << 8) | buff[2];
    magz = (((int16_t)buff[5]) << 8) | buff[4];

    mag[0] = magx;
    mag[1] = magy;
    mag[2] = magz;
}

void getRawGyroAccel(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz){
    uint8_t buffer[14];
    I2CReadBytes(MPU_CHIPADR, ACCEL_X_OUTPUT_MSB, buffer, 14, TIMEOUT_I2C);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

void getRawGyroAccelMag(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz){
    uint8_t buff[6];
    getRawGyroAccel(ax, ay, az, gx, gy, gz);

    I2CWriteByte(MPU_CHIPADR, INT_PIN_CFG, 0x02);
    delay(10);
    I2CWriteByte(MAG_CHIPADR, 0x0A, 0x01);
    delay(10);
    I2CReadBytes(MAG_CHIPADR, MAG_X_OUTPUT_LSB, buff, 6, TIMEOUT_I2C);
    
    *mx = (((int16_t)buff[1]) << 8) | buff[0];
    *my = (((int16_t)buff[3]) << 8) | buff[2];
    *mz = (((int16_t)buff[5]) << 8) | buff[4];
}

void setGyroXOffsetUser(int16_t binary){
    int8_t buff[2];
    buff[0] = binary & 0xFF;
    buff[1] = binary >> 8;
    I2CWriteByte(MPU_CHIPADR, GYRO_X_OFFSET_MSB, buff[1]);
    I2CWriteByte(MPU_CHIPADR, GYRO_X_OFFSET_LSB, buff[0]);
}

void setGyroYOffsetUser(int16_t binary){
    int8_t buff[2];
    buff[0] = binary & 0xFF;
    buff[1] = binary >> 8;
    I2CWriteByte(MPU_CHIPADR, GYRO_Y_OFFSET_MSB, buff[1]);
    I2CWriteByte(MPU_CHIPADR, GYRO_Y_OFFSET_LSB, buff[0]);
}

void setGyroZOffsetUser(int16_t binary){
    int8_t buff[2];
    buff[0] = binary & 0xFF;
    buff[1] = binary >> 8;
    I2CWriteByte(MPU_CHIPADR, GYRO_Z_OFFSET_MSB, buff[1]);
    I2CWriteByte(MPU_CHIPADR, GYRO_Z_OFFSET_LSB, buff[0]);
}

void setAccelXOffsetUser(int16_t binary){
    int8_t buff[2];
    buff[0] = binary & 0xFF;
    buff[1] = binary >> 8;
    I2CWriteByte(MPU_CHIPADR, ACCEL_X_OFFSET_MSB, buff[1]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_X_OFFSET_LSB, buff[0]);
}

void setAccelYOffsetUser(int16_t binary){
    int8_t buff[2];
    buff[0] = binary & 0xFF;
    buff[1] = binary >> 8;
    I2CWriteByte(MPU_CHIPADR, ACCEL_Y_OFFSET_MSB, buff[1]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_Y_OFFSET_LSB, buff[0]);
}

void setAccelZOffsetUser(int16_t binary){
    int8_t buff[2];
    buff[0] = binary & 0xFF;
    buff[1] = binary >> 8;
    I2CWriteByte(MPU_CHIPADR, ACCEL_Z_OFFSET_MSB, buff[1]);
    I2CWriteByte(MPU_CHIPADR, ACCEL_Z_OFFSET_LSB, buff[0]);
}

void mpuAccelGyroTest(){
  int16_t ax, ay, az, gx, gy, gz;
  getRawGyroAccel(&ax, &ay, &az, &gx, &gy, &gz);
  Serial2.println("Accel");
  Serial2.print(" X: ");
  Serial2.print(ax);
  Serial2.print(" Y: ");
  Serial2.print(ay);
  Serial2.print(" Z: ");
  Serial2.println(az);
  Serial2.println("Gyro");
  Serial2.print(" X: ");
  Serial2.print(gx);
  Serial2.print(" Y: ");
  Serial2.print(gy);
  Serial2.print(" Z: ");
  Serial2.println(gz);
  Serial2.println();
}

void mpuAccelGyroMagTest(){
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  getRawGyroAccelMag(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Serial2.println("Accel");
  Serial2.print(" X: ");
  Serial2.print(ax);
  Serial2.print(" Y: ");
  Serial2.print(ay);
  Serial2.print(" Z: ");
  Serial2.println(az);
  Serial2.println("Gyro");
  Serial2.print(" X: ");
  Serial2.print(gx);
  Serial2.print(" Y: ");
  Serial2.print(gy);
  Serial2.print(" Z: ");
  Serial2.println(gz);
  Serial2.println("Mag");
  Serial2.print(" X: ");
  Serial2.print(mx);
  Serial2.print(" Y: ");
  Serial2.print(my);
  Serial2.print(" Z: ");
  Serial2.println(mz);
  Serial2.println();
}