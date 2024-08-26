#include "gps.h"

TinyGPSPlus gps;
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPSSeriPort.available())
      gps.encode(GPSSeriPort.read());
  } while (millis() - start < ms);
}

void getGPSData(float *enlem, float *boylam, float *irtifa, int16_t *satellite){
  float enlemYedek, boylamYedek, irtifaYedek, hizYedek;
  int16_t satYedek;
  smartDelay(1000);
  enlemYedek = gps.location.lat();
  boylamYedek = gps.location.lng();
  satYedek = (int16_t)gps.satellites.value();
  irtifaYedek = gps.altitude.meters();

  if(gps.location.isValid()){
    *enlem = enlemYedek;
    *boylam = boylamYedek;
  }

  if(gps.altitude.isValid()){
    *irtifa = irtifaYedek;
  }

  if(gps.satellites.isValid()){
    *satellite = satYedek;
  }
}