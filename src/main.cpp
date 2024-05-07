#include <Arduino.h>
#include "bmp2xx.h"
#include "rf_main.h"
#include <MPU9250_WE.h>
#define MPU9250_ADDR 0x68
#define RFTEST
#define SERIPORTRF

//HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial2(PB7, PB6);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

bool RampaDeger = true;
bool IrtifaSinir = false;
bool KurtarmaSistemi = false;
float rampaDegerIrtifa = 0;
float altitude;
xyzFloat gValue;
float pitch, roll;

float altbuf[40];
float buftoplam = 0;
float bufort = 0;
int buffcnt = 0;

void anaAlg(float *alt, xyzFloat *g, float *pitch, float *roll);
void anaAlgSetup(void);
float rampaDegerFonksiyonu(void);

void setup()
{
  #ifdef SENSORTEST
    Serial2.begin(9600);
    while (!Serial2)
    {
    }
    Serial2.println("Seri port aktiflestirildi....");
    delay(1000);

    // rfInit();
    // Serial2.println("RF port aktiflestirildi....");
    delay(1000);
    I2Cinit(PB7, PB6);
    Serial2.println("I2C port aktiflestirildi....");
    delay(1000);
    bmpInit();
    Serial2.println("Bmp port aktiflestirildi....");
    delay(1000);
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
    delay(1000);
    Serial2.println("Position you MPU9250 flat and don't move it - calibrating...");
    delay(1000);
    myMPU9250.autoOffsets();
    Serial2.println("Done!");
    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
    myMPU9250.setSampleRateDivider(5);
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
    delay(200);
    Serial2.println("Rampa Deger Fonksiyonu Calistiriliyor...");
    anaAlgSetup();
    Serial2.print("Rampa Irtifa Degeri: ");
    Serial2.println(rampaDegerIrtifa);
  #endif

  #ifdef RFTEST
    #ifdef SERIPORTRF
      Serial2.begin(9600);
      while(!Serial2);
      Serial2.println("Seri port aktiflestirildi...");
    #endif
    delay(1000);
    rfInit();
    #ifdef SERIPORTRF
      Serial2.println("RF port aktiflestirildi...");
    #endif
    delay(1000);
  #endif
}

void loop()
{
  delay(50);

  denemeHaberlesmeReceiver();
  Serial2.println("Mesaj Loop...");
  delay(1000);
  #ifdef SENSORTEST
    altitude = getAltitudeReal();
    gValue = myMPU9250.getGValues();
    pitch = myMPU9250.getRoll();
    roll = myMPU9250.getPitch();

    anaAlg(&altitude, &gValue, &pitch, &roll);

    #ifdef SERIPORTMODE_ON
      Serial2.print("Altitude: ");
      Serial2.println(altitude);
      Serial2.print("G in z: ");
      Serial2.println(gValue.z);
      Serial2.print("Pitch: ");
      Serial2.println(pitch);
      Serial2.print("Roll: ");
      Serial2.println(roll);
      Serial2.print("Rampa Deger: ");
      Serial2.println(RampaDeger);
      Serial2.print("Irtifa Sinir: ");
      Serial2.println(IrtifaSinir);
      Serial2.print("Kurtarma Sistemi: ");
      Serial2.println(KurtarmaSistemi);
      Serial2.print("BufOrt: ");
      Serial2.println(bufort);
      Serial2.println();
    #endif
  #endif
}

float rampaDegerFonksiyonu(void)
{
  int32_t toplam = 0, pres, temp;
  float alt, ort;
  for (int i = 0; i < 300; i++)
  {
    alt = getAltitudeReal();
    toplam += alt;
    delay(50);
  }
  ort = toplam / (300 * 1.0);
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
    if (*alt > rampaDegerIrtifa + 0.5)
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
      if (*alt > rampaDegerIrtifa + 0.5)
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
        if (bufort > altitude && ((*pitch > 75 || *pitch < -75) || (*roll > 75 || *roll < -75)))
        {
          Serial2.println("Kurtarma Sistemi Ateslendi...");
          Serial2.println("Kurtarma Sistemi Ateslendi...");
          Serial2.println("Kurtarma Sistemi Ateslendi...");
          KurtarmaSistemi = true;
        }

        buftoplam = 0;

        if (buffcnt == 40)
        {
          buffcnt = 0;
        }

        altbuf[buffcnt++] = altitude;

        for (int i = 0; i < 40; i++)
        {
          buftoplam += altbuf[i];
        }

        bufort = buftoplam / 40.0;

        
      }
    }
  }
}
