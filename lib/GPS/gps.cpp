#include "gps.h"

TinyGPS gps;

static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (GPSSeriPort.available())
      gps.encode(GPSSeriPort.read());
  } while (millis() - start < ms);
}

void getGPSData(float *enlem, float *boylam, float *irtifa = (float *)0, float *hiz = (float *)0){
  float enlemYedek, boylamYedek, irtifaYedek, hizYedek;
  smartdelay(1000);
  gps.f_get_position(&enlemYedek, &boylamYedek);
  irtifaYedek = gps.f_altitude();
  hizYedek = gps.f_speed_mps();

  if(enlemYedek != TinyGPS::GPS_INVALID_F_ANGLE && boylamYedek != TinyGPS::GPS_INVALID_F_ANGLE){
    *enlem = enlemYedek;
    *boylam = boylamYedek;
  }

  if(irtifaYedek != TinyGPS::GPS_INVALID_ALTITUDE){
    *irtifa = irtifaYedek;
  }

  if(hizYedek != TinyGPS::GPS_INVALID_F_SPEED){
    *hiz = hizYedek;
  }
}

void gpsDatas(float *enlem, float *boylam, int *irtifa) {
  unsigned long age;
  smartdelay(1000);
  DEBUG_PRINTLN();

  uint8_t sat = gps.satellites();
  if (sat == TinyGPS::GPS_INVALID_SATELLITES) {
    DEBUG_PRINTLN(F("Gecersiz Uydu Sayisi"));
  } else {
    DEBUG_PRINT(F("Uydu Sayisi: "));
    DEBUG_PRINTLN(sat);
  }

  gps.f_get_position(enlem, boylam, &age);
  if (*enlem == TinyGPS::GPS_INVALID_F_ANGLE || *boylam == TinyGPS::GPS_INVALID_F_ANGLE) {
    DEBUG_PRINTLN(F("Gecersiz Konum Verisi"));
  } else {
    DEBUG_PRINT(F("Enlem: "));
    DEBUG_PRINTLN(*enlem, 6);
    DEBUG_PRINT(F("Boylam: "));
    DEBUG_PRINTLN(*boylam, 6);
  }

  *irtifa = gps.f_altitude();
  if (*irtifa == TinyGPS::GPS_INVALID_ALTITUDE) {
    DEBUG_PRINTLN("Gecersiz Rakim Verisi");
  } else {
    DEBUG_PRINT("Rakim: ");
    DEBUG_PRINTLN(*irtifa);
  }

  int speed = gps.f_speed_kmph();
  if (speed == TinyGPS::GPS_INVALID_F_SPEED) {
    DEBUG_PRINTLN("Gecersiz Hiz Verisi");
  } else {
    DEBUG_PRINT("Hizi: ");
    DEBUG_PRINTLN(speed);
  }

  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age2;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age2);
  if (year == TinyGPS::GPS_INVALID_DATE) {
    DEBUG_PRINTLN(F("Gecersiz Tarih Verisi"));
  } else {
    hour = (hour + 3) % 24; // UTC+3 saat dilimi için düzeltme
    if(hour == 0) {
      day++;
    }

    DEBUG_PRINT(F("Tarih: "));
    DEBUG_PRINT(day);
    DEBUG_PRINT(F("/"));
    DEBUG_PRINT(month);
    DEBUG_PRINT(F("/"));
    DEBUG_PRINTLN(year);

    DEBUG_PRINT(F("Saat: "));
    DEBUG_PRINT(hour < 10 ? F("0") : F(""));
    DEBUG_PRINT(hour);
    DEBUG_PRINT(F(":"));
    DEBUG_PRINT(minute < 10 ? F("0") : F(""));
    DEBUG_PRINT(minute);
    DEBUG_PRINT(F(":"));
    DEBUG_PRINT(second < 10 ? F("0") : F(""));
    DEBUG_PRINTLN(second);
  }
}