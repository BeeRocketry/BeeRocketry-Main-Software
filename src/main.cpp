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
  bmpInit();
  Serial2.println("BMP Port Aktifleştirildi.");
  delay(1000);
}

void loop() {
  mpuAccelGyroTest();
  bmpTest();
  delay(1000);
}