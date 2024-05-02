#include <Arduino.h>
#include "bmp2xx.h"

HardwareSerial Serial2(PA3, PA2);
#define seaLevelhPa 1013.25

void setup() {
  delay(3000);
  Serial2.begin(9600);
  while(!Serial2){

  }
  Serial2.println("Seri Port Aktifleştirildi.");
  delay(1000);
  Wire.setSCL(PB8);
  Wire.setSDA(PB7);
  Wire.begin();
  Serial2.println("I2C Port Aktifleştirildi.");
  delay(1000);
  bmpInit();
  Serial2.println("BMP Port Aktifleştirildi.");
  delay(1000);
}

void loop() {
  int32_t rawTemp, rawPres, temp, pres;
  int32_t tfine;
  int32_t altitude;
  float preshpa, tempc;

  getraws(&rawPres, &rawTemp);

  Serial2.println("Raw Degerler Alindi....");
  Serial2.print("rawTemp: ");
  Serial2.println(rawTemp);
  Serial2.print("rawPres: ");
  Serial2.println(rawPres);

  temp = getCompensatedTemp(rawTemp, &tfine);
  pres = getCompensatedPres(rawPres, tfine);
  preshpa = pres * 1.0 / 100;
  tempc = temp * 1.0 / 100;

  altitude = 44330 * (1.0 - pow(preshpa / seaLevelhPa, 0.1903));

  Serial2.println("Compensated Degerler Hesaplandi....");
  Serial2.print("Temp: ");
  Serial2.print(tempc);
  Serial2.println(" C");
  Serial2.print("Pres: ");
  Serial2.print(preshpa);
  Serial2.println(" Pa");
  Serial2.print("Altitude: ");
  Serial2.println(altitude);
  Serial2.println();
  Serial2.println();
  Serial2.println();

  delay(1000);
}