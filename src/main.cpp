#include <Arduino.h>
#include "rf_main.h"

void setup() {
  Serial.begin(9600);
  while(!Serial){

  }
  delay(100);
  rfInit();
}

void loop() {
  
}