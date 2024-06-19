#include <Arduino.h>
#include "bmp2xx.h"

// RF Kütüphanesi için makrolar
#define E32_TTL_1W
#define FREQUENCY_433
#include <LoRa_E32.h>
#include <MPU9250_WE.h>

#define MPU9250_ADDR 0x68

// Verici devre için kullanılacak makrolar
#define SENSOR
#define SENSOR_TRANSMITTER
#define ANA_ALGORITMA

// Alıcı devre için kullanılacak makrolar
// #define RECEIVER

// RF Modülü için AUX Pin Makrosu
#define AUXPIN PB1

// Aviyonik test için kullanılacak pinlerin makroları
#define BuzzerPIN PB0
#define RampaLED PA7
#define IrtifaLED PA6
#define KurtarmaLED PA5

// Float byte dizisi arasında dönüşüm yapmak için
union float2binary
{
  float floating;
  byte binary[4];
};

// Alıcı devrenin obje tanımları ve fonksiyon prototipleri
#ifdef RECEIVER
  HardwareSerial Serial2(PB7, PB6);
  HardwareSerial UART1(PA3, PA2);
  LoRa_E32 e32ttl1w(&UART1, AUXPIN);

  void setsMainOpt();
  void printParameters(struct Configuration conf);
#endif

// Sensör devresinin obje tanımları
#ifdef SENSOR
  HardwareSerial Serial2(PA3, PA2);

  // Verici devrenin obje tanımları ve fonksiyon prototipleri
  #ifdef SENSOR_TRANSMITTER
    HardwareSerial UART1(PB7, PB6);
    LoRa_E32 e32ttl1w(&UART1, AUXPIN);

    void setsMainOpt();
    void printParameters(struct Configuration conf);
  #endif
#endif

// Ana algoritma devresi için genel obje tanımlamaları değişken tanımları
#ifdef ANA_ALGORITMA && SENSOR
  MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

  bool RampaDeger = true;
  bool IrtifaSinir = false;
  bool KurtarmaSistemi = false;
  float rampaDegerIrtifa = 0;
  float altitude;
  int32_t sicaklik, basinc;
  xyzFloat gValue;
  xyzFloat gyro;
  float pitch, roll;

  float altbuf[40];
  float buftoplam = 0;
  float bufort = 0;
  int buffcnt = 0;
  int lasttime;

  void anaAlg(float *alt, xyzFloat *g, float *pitch, float *roll);
  void anaAlgSetup(void);
  float rampaDegerFonksiyonu(void);
#endif

void setup()
{
  #ifdef SENSOR
    Serial2.begin(9600);
    while (!Serial2)
    {
    }
    Serial2.println("Seri port aktiflestirildi....");
    delay(2000);

    #ifdef SENSOR_TRANSMITTER
      e32ttl1w.begin();
      setsMainOpt();
      Serial2.println("RF port aklestirildi....");
      delay(2000);
    #endif

    I2Cinit(PB9, PB8);
    Serial2.println("I2C port aktiflestirildi....");
    delay(2000);

    bmpInit();
    Serial2.println("Bmp port aktiflestirildi....");
    delay(2000);

    if (myMPU9250.init())
    {
      Serial2.println("MPU port aktiflestirilemedi....");
    }
    else
    {
      Serial2.println("MPU port aktiflestirildi....");
    }

    if (myMPU9250.initMagnetometer())
    {
      Serial2.println("Mag port aktiflestirilemedi....");
    }
    else
    {
      Serial2.println("Mag port aktiflestirildi....");
    }
    delay(2000);

    Serial2.println("MPU Kalibre ediliyor..");
    delay(2000);
    myMPU9250.autoOffsets();
    Serial2.println("Tamamlandi!");

    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
    myMPU9250.setSampleRateDivider(5);
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
    delay(1000);
    
    Serial2.println("Rampa Deger Fonksiyonu Calistiriliyor...");
    anaAlgSetup();
    Serial2.print("Rampa Irtifa Degeri: ");
    Serial2.println(rampaDegerIrtifa);
  #endif

  #ifdef RECEIVER
    Serial2.begin(9600);
    while (!Serial2)
      ;
    Serial2.println("Seri port aktiflestirildi...");
    delay(1000);
    e32ttl1w.begin();
    setsMainOpt();
    Serial2.println("RF port aktiflestirildi...");
    delay(1000);
  #endif
}

void loop()
{
  delay(50);
  #ifdef SENSOR
    altitude = getAltitudeReal(&sicaklik, &basinc);
    gValue = myMPU9250.getGValues();
    roll = myMPU9250.getRoll();
    pitch = myMPU9250.getPitch();
    gyro = myMPU9250.getGyrValues();
    Serial2.print("Irtifa: ");
    Serial2.println(altitude);
    Serial2.print("Pitch: ");
    Serial2.println(pitch);
    Serial2.print("Roll: ");
    Serial2.println(roll);
    Serial2.println("Ivme:");
    Serial2.print(" X: ");
    Serial2.println(gValue.x);
    Serial2.print(" Y: ");
    Serial2.println(gValue.y);
    Serial2.print(" Z: ");
    Serial2.println(gValue.z);
    Serial2.println();

    #ifdef SENSOR_TRANSMITTER
      float2binary hi;

      hi.floating = gValue.x;
      UART1.write(hi.binary, 4);

      Serial2.print("Ivme X: ");
      Serial2.println(*(float *)hi.binary);

      hi.floating = altitude;
      UART1.write(hi.binary, 4);

      Serial2.print("Irtifa: ");
      Serial2.println(*(float *)hi.binary);

      hi.floating = pitch;
      UART1.write(hi.binary, 4);

      Serial2.print("Pitch: ");
      Serial2.println(*(float *)hi.binary);

      Serial2.println();
    #endif

    #ifdef ANA_ALGORITMA
      anaAlg(&altitude, &gValue, &pitch, &roll);
    #endif

  #endif

  #ifdef RECEIVER
    float2binary den;
    float sicaklik, irtifa;
    Serial2.println("Mesaj Loop..");
    byte arr[4];
    int cnt = 1;
    while (UART1.available())
    {
      UART1.readBytes(arr, sizeof(float));
      if (cnt == 1)
      {
        Serial2.print("Ivme X: ");
      }
      else if (cnt == 2)
      {
        Serial2.print("Irtifa: ");
      }
      else if (cnt == 3)
      {
        Serial2.print("Pitch: ");
      }
      cnt++;
      den.binary[3] = arr[3];
      den.binary[2] = arr[2];
      den.binary[1] = arr[1];
      den.binary[0] = arr[0];
      Serial2.println(den.floating);
    }
    Serial2.println();
  #endif
}

#ifdef ANA_ALGORITMA
  float rampaDegerFonksiyonu(void)
  {
    int32_t toplam = 0, pres, temp;
    float alt, ort;
    for (int i = 0; i < 5; i++)
    {
      alt = getAltitudeReal(&sicaklik, &basinc); // GetAltitudeReal irtifa değerinin döndüren fonksiyondur
      toplam += alt;
      delay(50);
    }
    ort = toplam / (5 * 1.0);
    return ort;
  }

  void anaAlgSetup(void)
  {
    rampaDegerIrtifa = rampaDegerFonksiyonu();
  }

  void anaAlg(float *alt, xyzFloat *g, float *pitch, float *roll)
  {
    if (RampaDeger == true)
    {
      if (*alt > rampaDegerIrtifa + 400)
      {
        RampaDeger = false;
        Serial2.println("Rampa Deger Degeri false oldu...");
        Serial2.println("Rampa Deger Degeri false oldu...");
        Serial2.println("Rampa Deger Degeri false oldu...");
      }
    }

    else
    {
      if (IrtifaSinir == false)
      {
        if (*alt > rampaDegerIrtifa + 600)
        {
          IrtifaSinir = true;
          Serial2.println("Irtifa Sinir Degeri true oldu...");
          Serial2.println("Irtifa Sinir Degeri true oldu...");
          Serial2.println("Irtifa Sinir Degeri true oldu...");
        }
      }

      else
      {
        if (KurtarmaSistemi == false)
        {
          if (bufort - 200 > altitude && ((*pitch > 75 || *pitch < -75) || (*roll > 75 || *roll < -75)))
          {
            Serial2.println("Kurtarma Sistemi Ateslendi...");
            Serial2.println("Kurtarma Sistemi Ateslendi...");
            Serial2.println("Kurtarma Sistemi Ateslendi...");
            if (KurtarmaSistemi == false)
            {
              lasttime = millis();
            }

            KurtarmaSistemi = true;
          }

          buftoplam = 0;

          if (buffcnt == 20)
          {
            buffcnt = 0;
          }

          altbuf[buffcnt++] = altitude;

          for (int i = 0; i < 20; i++)
          {
            buftoplam += altbuf[i];
          }

          bufort = buftoplam / 20.0;
        }
      }
    }
  }
#endif

#ifdef SENSOR_TRANSMITTER || RECEIVER
  void setsMainOpt()
  {
    // Bu Function RF'in başlangıçtaki temel ayarlarını yapmaktadır.

    ResponseStructContainer ayar;
    ayar = e32ttl1w.getConfiguration();

    Configuration conf = *(Configuration *)ayar.data;

    conf.ADDL = 0x3;
    conf.ADDH = 0x8;
    conf.CHAN = 23;

    conf.OPTION.fec = FEC_1_ON;
    conf.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    conf.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    conf.OPTION.wirelessWakeupTime = WAKE_UP_250;
    conf.OPTION.transmissionPower = POWER_30;

    conf.SPED.airDataRate = AIR_DATA_RATE_010_24;
    conf.SPED.uartBaudRate = UART_BPS_9600;
    conf.SPED.uartParity = MODE_00_8N1;

    ResponseStatus rs = e32ttl1w.setConfiguration(conf, WRITE_CFG_PWR_DWN_SAVE);

    Serial2.println("Config: " + rs.getResponseDescription());

    printParameters(conf);
    ayar.close();
  }

  void printParameters(struct Configuration conf)
  {
    // Bu Fonksiyon RF modülünün tüm ayarlarını printliyor.

    Serial2.println("----------------------------------------");

    Serial2.print(F("HEAD : "));
    Serial2.print(conf.HEAD, BIN);
    Serial2.print(" ");
    Serial2.print(conf.HEAD, DEC);
    Serial2.print(" ");
    Serial2.println(conf.HEAD, HEX);
    Serial2.println(F(" "));
    Serial2.print(F("AddH : "));
    Serial2.println(conf.ADDH, BIN);
    Serial2.print(F("AddL : "));
    Serial2.println(conf.ADDL, BIN);
    Serial2.print(F("Chan : "));
    Serial2.print(conf.CHAN, DEC);
    Serial2.print(" -> ");
    Serial2.println(conf.getChannelDescription());
    Serial2.println(F(" "));
    Serial2.print(F("SpeedParityBit     : "));
    Serial2.print(conf.SPED.uartParity, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.SPED.getUARTParityDescription());
    Serial2.print(F("SpeedUARTDatte  : "));
    Serial2.print(conf.SPED.uartBaudRate, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.SPED.getUARTBaudRate());
    Serial2.print(F("SpeedAirDataRate   : "));
    Serial2.print(conf.SPED.airDataRate, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.SPED.getAirDataRate());

    Serial2.print(F("OptionTrans        : "));
    Serial2.print(conf.OPTION.fixedTransmission, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getFixedTransmissionDescription());
    Serial2.print(F("OptionPullup       : "));
    Serial2.print(conf.OPTION.ioDriveMode, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getIODroveModeDescription());
    Serial2.print(F("OptionWakeup       : "));
    Serial2.print(conf.OPTION.wirelessWakeupTime, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getWirelessWakeUPTimeDescription());
    Serial2.print(F("OptionFEC          : "));
    Serial2.print(conf.OPTION.fec, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getFECDescription());
    Serial2.print(F("OptionPower        : "));
    Serial2.print(conf.OPTION.transmissionPower, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getTransmissionPowerDescription());

    Serial2.println("----------------------------------------");
  }
#endif