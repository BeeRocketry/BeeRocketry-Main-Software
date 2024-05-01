#include <Arduino.h>
#include "rf_main.h"
#include "I2C.h"
#include "bmp2xx.h"

void setup() {
  Serial.begin(9600);
  while(!Serial){

  }
  delay(100);
  I2Cinit();
  bmpInit();
}

void loop() {
  int32_t tempraw = 0, presraw = 0, temp = 0, pres = 0, tfine = 0;

  tempraw = getRawTemp();
  temp = getCompensatedTemp(tempraw, &tfine);
  presraw = getRawPres();
  pres = getCompensatedPres(presraw, tfine);
  
  Serial.print("Sicaklik: ");
  Serial.println(temp);
  Serial.print("Basinc: ");
  Serial.println(pres);

  delay(1000);
}