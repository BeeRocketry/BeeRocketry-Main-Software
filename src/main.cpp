#include <Arduino.h>
#include "bmp2xx.h"
#include "rf_main.h"
#include "MPU9250.h"

HardwareSerial Serial2(PA3, PA2);

void setup() {
  delay(3000);
  Serial2.begin(9600);
  while(!Serial2){

  }
  Serial2.println("Seri Port Aktifleştirildi.");
  delay(1000);
  I2Cinit(PB7, PB8);
  Serial2.println("I2C Port Aktifleştirildi.");
  delay(1000);
  mpuInit();
  Serial2.println("MPU Port Aktifleştirildi.");
  delay(1000);

  Serial2.println(getDeviceID(), HEX);
  delay(1000);
}

void loop() {
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
  Serial2.println();
  delay(1000);
}