#include <Arduino.h>
#include "rf_main.h"
#include "I2C.h"
#include "bmp2xx.h"

void setup() {
  Serial.begin(9600);
  while(!Serial){

  }
  delay(100);
  rfInit();
  I2Cinit();
  bmpInit();
}

void loop() {
  haberlesmeTestTransmitter();
}